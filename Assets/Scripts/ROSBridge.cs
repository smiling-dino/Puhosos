using Unity.MLAgents;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

/// <summary>
/// Publishes commands produced by the ML-Agents policy to the physical GFS-X robot.
///
/// The bridge reads the actuator state already applied by TrackController and
/// GripperController. Therefore ROS receives the same commands after dead zones,
/// PWM slew limiting and the current Z-fold manipulator logic.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(TrackController))]
public sealed class ROSBridge : MonoBehaviour
{
    [Header("Activation")]
    [Tooltip("Enable only on the single robot used for physical inference.")]
    [SerializeField] private bool bridgeEnabled = false;
    [Tooltip("Allow enabling this bridge in standalone builds with --ros-bridge.")]
    [SerializeField] private bool enableFromCommandLine = true;
    [SerializeField] private bool disableWhileTrainerConnected = true;
    [SerializeField] private bool enforceSinglePublisher = true;

    [Header("ROS Connection Override")]
    [SerializeField] private bool configureConnectionFromCommandLine = true;
    [SerializeField] private string enableArgument = "--ros-bridge";
    [SerializeField] private string rosIpArgument = "--ros-ip";
    [SerializeField] private string rosPortArgument = "--ros-port";

    [Header("JSON Start Gate")]
    [Tooltip("Open a local TCP/HTTP listener that accepts /activate only after a start JSON.")]
    [SerializeField] private bool activationServerEnabled = false;
    [Tooltip("Allow enabling the JSON start listener in standalone builds with --activation-server.")]
    [SerializeField] private bool activationServerFromCommandLine = true;
    [Tooltip("Keep ROS publishing disabled until /activate receives a valid start JSON.")]
    [SerializeField] private bool requireStartJsonBeforePublishing = true;
    [SerializeField] private string activationServerArgument = "--activation-server";
    [SerializeField] private string activationPortArgument = "--activation-port";
    [SerializeField] private string activationListenAddress = "0.0.0.0";
    [SerializeField] private int activationListenPort = 8765;
    [SerializeField] private string healthPath = "/health";
    [SerializeField] private string setTargetPath = "/set_target";
    [SerializeField] private string activatePath = "/activate";

    [Header("Robot Components")]
    [SerializeField] private RobotBrain robotBrain;
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private Transform cameraServo;

    [Header("ROS Topics")]
    [SerializeField] private string cmdVelTopic = "/gfsx/cmd_vel";
    [SerializeField] private string cameraCommandTopic = "/gfsx/camera_cmd";
    [SerializeField] private string gripperCommandTopic = "/gfsx/gripper_cmd";
    [SerializeField] private string liftCommandTopic = "/gfsx/lift_cmd";

    [Header("Physical Limits")]
    [Min(0.001f)]
    [SerializeField] private float maxLinearSpeedMetersPerSecond = 0.08f;
    [Min(0.001f)]
    [SerializeField] private float maxAngularSpeedRadiansPerSecond = 0.45f;
    [Min(0.001f)]
    [SerializeField] private float cameraTurnSpeedDegreesPerSecond = 25f;

    [Header("Filtering")]
    [Range(0.01f, 1f)]
    [SerializeField] private float emaAlpha = 0.25f;
    [Range(0f, 0.25f)]
    [SerializeField] private float commandDeadzone = 0.03f;
    [Min(0f)]
    [SerializeField] private float minimumPublishIntervalSeconds = 0.10f;

    [Header("Manipulator Safety")]
    [Min(0f)]
    [SerializeField] private float gripperCommandCooldownSeconds = 0.70f;
    [Min(0f)]
    [SerializeField] private float liftCommandCooldownSeconds = 0.20f;
    [SerializeField] private bool blockLiftUntilGripperClosed = true;

    [Header("Fail-safe")]
    [Min(0.05f)]
    [SerializeField] private float policyWatchdogTimeoutSeconds = 0.5f;
    [SerializeField] private bool publishInitialGripperState = true;

    private static ROSBridge activePublisher;
    private static ROSBridge activeActivationServer;

    private ROSConnection ros;
    private TcpListener activationListener;
    private CancellationTokenSource activationServerCancellation;
    private Task activationServerTask;
    private bool initialized;
    private bool ownsPublisherSlot;
    private bool ownsActivationServerSlot;
    private bool watchdogLatched;
    private bool gripperStateInitialized;
    private bool waitingForStartJson;
    private volatile bool startRequestedByApi;
    private readonly object activationStateLock = new object();

    private int lastAgentStep = int.MinValue;
    private int lastLiftCommand = int.MinValue;
    private bool lastClosedState;

    private float smoothedLinear;
    private float smoothedAngular;
    private float previousLiftNormalized;
    private float initialCameraYawDegrees;
    private float previousCameraYawDegrees;
    private float lastPolicyStepRealtime;
    private float lastPublishRealtime = float.NegativeInfinity;
    private float lastGripperCommandRealtime = float.NegativeInfinity;
    private float lastLiftCommandRealtime = float.NegativeInfinity;
    private float lastCameraSampleRealtime;
    private string runtimeRosIpOverride;
    private int runtimeRosPortOverride = -1;
    private string lastTargetClass;

    private void Awake()
    {
        ApplyRuntimeConfiguration();
        ResolveReferences();
        CaptureInitialState();
        StartActivationServerIfNeeded();
    }

    private void Update()
    {
        if (!startRequestedByApi)
        {
            return;
        }

        startRequestedByApi = false;
        waitingForStartJson = false;
        SetBridgeEnabled(true, "valid start JSON received");
    }

    private void FixedUpdate()
    {
        if (!bridgeEnabled || !EnsureInitialized() || !CanPublish())
        {
            return;
        }

        float now = Time.realtimeSinceStartup;
        int currentStep = robotBrain.StepCount;

        if (currentStep != lastAgentStep)
        {
            lastAgentStep = currentStep;
            lastPolicyStepRealtime = now;
            watchdogLatched = false;

            if (now - lastPublishRealtime >= minimumPublishIntervalSeconds)
            {
                PublishAppliedCommands(now);
            }

            return;
        }

        if (!watchdogLatched &&
            now - lastPolicyStepRealtime > policyWatchdogTimeoutSeconds)
        {
            watchdogLatched = true;
            PublishEmergencyStop("ML-Agents policy stopped producing fresh steps");
        }
    }

    private void OnDisable()
    {
        StopActivationServer("ROSBridge disabled");
        Shutdown("ROSBridge disabled");
    }

    private void OnDestroy()
    {
        StopActivationServer("ROSBridge destroyed");
        Shutdown("ROSBridge destroyed");
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused)
        {
            PublishEmergencyStop("Unity application paused");
        }
    }

    private void OnApplicationQuit()
    {
        PublishEmergencyStop("Unity application quitting");
        StopActivationServer("Unity application quitting");
    }

    [ContextMenu("Publish emergency stop")]
    public void PublishEmergencyStop()
    {
        PublishEmergencyStop("manual request");
    }

    public void SetBridgeEnabled(bool enabled)
    {
        SetBridgeEnabled(enabled, "external request");
    }

    private void SetBridgeEnabled(bool enabled, string reason)
    {
        if (bridgeEnabled == enabled)
        {
            return;
        }

        bridgeEnabled = enabled;
        if (enabled)
        {
            watchdogLatched = false;
            lastPolicyStepRealtime = Time.realtimeSinceStartup;
            Debug.Log($"ROSBridge publishing enabled: {reason}.", this);
        }
        else
        {
            Shutdown($"ROSBridge disabled: {reason}");
        }
    }

    private void ResolveReferences()
    {
        robotBrain ??= GetComponent<RobotBrain>();
        trackController ??= GetComponent<TrackController>();
        gripperController ??= GetComponent<GripperController>();

        if (cameraServo == null)
        {
            Camera childCamera = GetComponentInChildren<Camera>(true);
            cameraServo = childCamera != null ? childCamera.transform : null;
        }
    }

    private void CaptureInitialState()
    {
        previousLiftNormalized = gripperController != null
            ? gripperController.LiftNormalized
            : 0f;
        lastClosedState = gripperController != null && gripperController.IsClosed;

        if (cameraServo != null)
        {
            initialCameraYawDegrees = cameraServo.localEulerAngles.y;
        }

        float now = Time.realtimeSinceStartup;
        lastPolicyStepRealtime = now;
        lastCameraSampleRealtime = now;
    }

    private bool EnsureInitialized()
    {
        if (initialized)
        {
            return true;
        }

        ResolveReferences();

        if (disableWhileTrainerConnected && Academy.Instance.IsCommunicatorOn)
        {
            bridgeEnabled = false;
            return false;
        }

        if (robotBrain == null || trackController == null)
        {
            Debug.LogError(
                "ROSBridge requires RobotBrain and TrackController on the robot root.",
                this
            );
            bridgeEnabled = false;
            return false;
        }

        if (enforceSinglePublisher &&
            activePublisher != null &&
            activePublisher != this)
        {
            Debug.LogError(
                $"ROS topics are already owned by '{activePublisher.name}'. " +
                "Keep ROSBridge disabled on cloned training arenas.",
                this
            );
            bridgeEnabled = false;
            return false;
        }

        try
        {
            if (enforceSinglePublisher)
            {
                activePublisher = this;
                ownsPublisherSlot = true;
            }

            ros = ROSConnection.GetOrCreateInstance();
            ApplyConnectionOverride(ros);
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
            ros.RegisterPublisher<Float32Msg>(cameraCommandTopic);
            ros.RegisterPublisher<Int32Msg>(gripperCommandTopic);
            ros.RegisterPublisher<Int32Msg>(liftCommandTopic);

            initialized = true;
            lastAgentStep = robotBrain.StepCount;
            lastPolicyStepRealtime = Time.realtimeSinceStartup;
            lastCameraSampleRealtime = lastPolicyStepRealtime;

            if (publishInitialGripperState && gripperController != null)
            {
                PublishGripperState(gripperController.IsClosed);
            }

            PublishDrive(0f, 0f, true);
            PublishCamera(0f);
            PublishLift(0);

            Debug.Log(
                $"ROSBridge initialized for GFS-X inference. " +
                $"Publishing '{cmdVelTopic}', '{cameraCommandTopic}', " +
                $"'{gripperCommandTopic}', '{liftCommandTopic}' to " +
                $"{ros.RosIPAddress}:{ros.RosPort}.",
                this
            );
            return true;
        }
        catch (System.Exception exception)
        {
            Debug.LogError(
                $"ROSBridge initialization failed: {exception.Message}",
                this
            );
            initialized = false;
            ros = null;
            ReleasePublisherSlot();
            return false;
        }
    }

    private bool CanPublish()
    {
        if (!disableWhileTrainerConnected ||
            !Academy.Instance.IsCommunicatorOn)
        {
            return true;
        }

        if (!watchdogLatched)
        {
            watchdogLatched = true;
            PublishEmergencyStop("mlagents-learn communicator is connected");
        }

        return false;
    }

    private void PublishAppliedCommands(float now)
    {
        GetNormalizedDriveCommands(out float linear, out float angular);
        PublishDrive(linear, angular, false);
        PublishCameraFromServo(now);
        PublishGripperIfChanged();
        PublishLiftFromMechanism();
        lastPublishRealtime = now;
    }

    private void GetNormalizedDriveCommands(
        out float linear,
        out float angular)
    {
        float maxTrackCommand = Mathf.Max(
            0.001f,
            trackController.maxLinearCmd + Mathf.Abs(trackController.turnK)
        );

        float leftCommand =
            trackController.LeftPwmNormalized * maxTrackCommand;
        float rightCommand =
            trackController.RightPwmNormalized * maxTrackCommand;

        linear = Mathf.Clamp(
            ((leftCommand + rightCommand) * 0.5f) /
            Mathf.Max(0.001f, trackController.maxLinearCmd),
            -1f,
            1f
        );

        angular = Mathf.Abs(trackController.turnK) > 0.0001f
            ? Mathf.Clamp(
                ((leftCommand - rightCommand) * 0.5f) /
                trackController.turnK,
                -1f,
                1f
            )
            : 0f;
    }

    private void PublishDrive(
        float linearNormalized,
        float angularNormalized,
        bool forceHardStop)
    {
        bool hardStop = forceHardStop ||
            (Mathf.Abs(linearNormalized) <= commandDeadzone &&
             Mathf.Abs(angularNormalized) <= commandDeadzone);

        if (hardStop)
        {
            smoothedLinear = 0f;
            smoothedAngular = 0f;
        }
        else
        {
            smoothedLinear =
                emaAlpha * Mathf.Clamp(linearNormalized, -1f, 1f) +
                (1f - emaAlpha) * smoothedLinear;
            smoothedAngular =
                emaAlpha * Mathf.Clamp(angularNormalized, -1f, 1f) +
                (1f - emaAlpha) * smoothedAngular;
        }

        TwistMsg message = new TwistMsg();
        message.linear.x =
            smoothedLinear * maxLinearSpeedMetersPerSecond;
        message.angular.z =
            smoothedAngular * maxAngularSpeedRadiansPerSecond;
        ros.Publish(cmdVelTopic, message);
    }

    private void PublishCameraFromServo(float now)
    {
        if (cameraServo == null)
        {
            return;
        }

        float currentYaw = Mathf.DeltaAngle(
            initialCameraYawDegrees,
            cameraServo.localEulerAngles.y
        );
        float elapsed = Mathf.Max(0.001f, now - lastCameraSampleRealtime);
        float yawDelta = Mathf.DeltaAngle(
            previousCameraYawDegrees,
            currentYaw
        );

        PublishCamera(
            Mathf.Clamp(
                yawDelta /
                (cameraTurnSpeedDegreesPerSecond * elapsed),
                -1f,
                1f
            )
        );

        previousCameraYawDegrees = currentYaw;
        lastCameraSampleRealtime = now;
    }

    private void PublishCamera(float normalizedVelocity)
    {
        ros.Publish(
            cameraCommandTopic,
            new Float32Msg(Mathf.Clamp(normalizedVelocity, -1f, 1f))
        );
    }

    private void PublishGripperIfChanged()
    {
        if (gripperController == null)
        {
            return;
        }

        bool closed = gripperController.IsClosed;
        if (!gripperStateInitialized || closed != lastClosedState)
        {
            PublishGripperState(closed);
        }
    }

    private void PublishGripperState(bool closed)
    {
        float now = Time.realtimeSinceStartup;
        if (gripperStateInitialized &&
            now - lastGripperCommandRealtime < gripperCommandCooldownSeconds)
        {
            return;
        }

        ros.Publish(
            gripperCommandTopic,
            new Int32Msg(closed ? 1 : 2)
        );
        lastClosedState = closed;
        gripperStateInitialized = true;
        lastGripperCommandRealtime = now;
    }

    private void PublishLiftFromMechanism()
    {
        if (gripperController == null)
        {
            return;
        }

        float current = gripperController.LiftNormalized;
        float delta = current - previousLiftNormalized;
        previousLiftNormalized = current;

        int command = delta > 0.0001f
            ? 1
            : delta < -0.0001f
                ? 2
                : 0;

        if (blockLiftUntilGripperClosed && !gripperController.IsClosed)
        {
            command = 0;
        }

        PublishLift(command);
    }

    private void PublishLift(int command)
    {
        command = Mathf.Clamp(command, 0, 2);
        if (command == lastLiftCommand)
        {
            return;
        }

        float now = Time.realtimeSinceStartup;
        if (now - lastLiftCommandRealtime < liftCommandCooldownSeconds)
        {
            return;
        }

        ros.Publish(liftCommandTopic, new Int32Msg(command));
        lastLiftCommand = command;
        lastLiftCommandRealtime = now;
    }

    private void PublishEmergencyStop(string reason)
    {
        if (!initialized || ros == null)
        {
            return;
        }

        try
        {
            PublishDrive(0f, 0f, true);
            PublishCamera(0f);
            PublishLift(0);
            Debug.LogWarning($"ROSBridge emergency stop: {reason}.", this);
        }
        catch (System.Exception exception)
        {
            Debug.LogWarning(
                $"ROSBridge failed to publish stop: {exception.Message}",
                this
            );
        }
    }

    private void Shutdown(string reason)
    {
        if (initialized)
        {
            PublishEmergencyStop(reason);
        }

        initialized = false;
        ros = null;
        ReleasePublisherSlot();
    }

    private void ReleasePublisherSlot()
    {
        if (ownsPublisherSlot && activePublisher == this)
        {
            activePublisher = null;
        }

        ownsPublisherSlot = false;
    }

    private void StartActivationServerIfNeeded()
    {
        if (!activationServerEnabled)
        {
            return;
        }

        if (activeActivationServer != null &&
            activeActivationServer != this)
        {
            return;
        }

        if (activationListenPort <= 0 || activationListenPort > 65535)
        {
            Debug.LogError(
                $"Ignoring invalid activation listener port {activationListenPort}.",
                this
            );
            return;
        }

        try
        {
            IPAddress listenAddress = ResolveListenAddress(activationListenAddress);
            activationListener = new TcpListener(listenAddress, activationListenPort);
            activationListener.Start();
            activationServerCancellation = new CancellationTokenSource();
            activationServerTask = Task.Run(
                () => ActivationServerLoop(activationServerCancellation.Token)
            );

            activeActivationServer = this;
            ownsActivationServerSlot = true;

            Debug.Log(
                $"ROSBridge JSON start listener opened on " +
                $"{activationListenAddress}:{activationListenPort}. " +
                $"POST {activatePath} with {{\"start\":true}} to enable robot publishing.",
                this
            );
        }
        catch (Exception exception)
        {
            activationListener = null;
            activationServerCancellation?.Dispose();
            activationServerCancellation = null;
            activationServerTask = null;
            Debug.LogError(
                $"Failed to open ROSBridge JSON start listener: {exception.Message}",
                this
            );
        }
    }

    private void StopActivationServer(string reason)
    {
        bool hadServer =
            activationListener != null ||
            activationServerCancellation != null ||
            ownsActivationServerSlot;

        try
        {
            activationServerCancellation?.Cancel();
            activationListener?.Stop();
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                $"Failed to stop ROSBridge JSON start listener: {exception.Message}",
                this
            );
        }

        activationListener = null;
        activationServerCancellation?.Dispose();
        activationServerCancellation = null;
        activationServerTask = null;

        if (ownsActivationServerSlot && activeActivationServer == this)
        {
            activeActivationServer = null;
        }

        ownsActivationServerSlot = false;

        if (hadServer && !string.IsNullOrWhiteSpace(reason))
        {
            Debug.Log($"ROSBridge JSON start listener stopped: {reason}.", this);
        }
    }

    private async Task ActivationServerLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            TcpClient client = null;
            try
            {
                client = await activationListener.AcceptTcpClientAsync();
                _ = Task.Run(() => HandleActivationClient(client, token), token);
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (SocketException)
            {
                if (!token.IsCancellationRequested)
                {
                    Debug.LogWarning("ROSBridge JSON start listener socket closed unexpectedly.", this);
                }

                break;
            }
            catch (Exception exception)
            {
                client?.Close();
                if (!token.IsCancellationRequested)
                {
                    Debug.LogWarning(
                        $"ROSBridge JSON start listener error: {exception.Message}",
                        this
                    );
                }
            }
        }
    }

    private async Task HandleActivationClient(TcpClient client, CancellationToken token)
    {
        using (client)
        {
            client.ReceiveTimeout = 3000;
            client.SendTimeout = 3000;
            NetworkStream stream = client.GetStream();
            ApiResponse response;

            try
            {
                HttpRequest request = await ReadHttpRequest(stream);
                response = HandleApiRequest(request);
            }
            catch (Exception exception)
            {
                response = new ApiResponse(
                    400,
                    new JObject
                    {
                        ["ok"] = false,
                        ["error"] = exception.Message
                    }
                );
            }

            if (!token.IsCancellationRequested)
            {
                await WriteHttpResponse(stream, response);
            }
        }
    }

    private ApiResponse HandleApiRequest(HttpRequest request)
    {
        string route = NormalizeRoute(request.Path);

        if (string.Equals(request.Method, "GET", StringComparison.OrdinalIgnoreCase) &&
            string.Equals(route, NormalizeRoute(healthPath), StringComparison.OrdinalIgnoreCase))
        {
            string targetClass;
            lock (activationStateLock)
            {
                targetClass = lastTargetClass;
            }

            return new ApiResponse(
                200,
                new JObject
                {
                    ["ok"] = true,
                    ["bridgeEnabled"] = bridgeEnabled,
                    ["waitingForStart"] = waitingForStartJson,
                    ["lastTargetClass"] = targetClass ?? string.Empty
                }
            );
        }

        if (!string.Equals(request.Method, "POST", StringComparison.OrdinalIgnoreCase))
        {
            return new ApiResponse(
                405,
                new JObject
                {
                    ["ok"] = false,
                    ["error"] = "Only GET /health and POST JSON commands are supported."
                }
            );
        }

        if (string.Equals(route, NormalizeRoute(setTargetPath), StringComparison.OrdinalIgnoreCase))
        {
            return HandleSetTargetRequest(request.Body);
        }

        if (string.Equals(route, NormalizeRoute(activatePath), StringComparison.OrdinalIgnoreCase) ||
            route == "/")
        {
            return HandleActivateRequest(request.Body);
        }

        return new ApiResponse(
            404,
            new JObject
            {
                ["ok"] = false,
                ["error"] = $"Unknown path '{request.Path}'."
            }
        );
    }

    private ApiResponse HandleSetTargetRequest(string body)
    {
        JToken payload = ParseJsonBody(body);
        string className = TryFindStringField(payload, "class_name");
        lock (activationStateLock)
        {
            lastTargetClass = className;
        }

        return new ApiResponse(
            200,
            new JObject
            {
                ["ok"] = true,
                ["accepted"] = "set_target",
                ["class_name"] = className ?? string.Empty
            }
        );
    }

    private ApiResponse HandleActivateRequest(string body)
    {
        JToken payload = ParseJsonBody(body);
        if (!ContainsStartSignal(payload))
        {
            return new ApiResponse(
                400,
                new JObject
                {
                    ["ok"] = false,
                    ["error"] = "JSON must contain a truthy 'start' field or state:'start'."
                }
            );
        }

        startRequestedByApi = true;
        return new ApiResponse(
            200,
            new JObject
            {
                ["ok"] = true,
                ["accepted"] = "activate",
                ["state"] = "start"
            }
        );
    }

    private static async Task<HttpRequest> ReadHttpRequest(NetworkStream stream)
    {
        using StreamReader reader = new StreamReader(
            stream,
            Encoding.UTF8,
            false,
            4096,
            true
        );

        string requestLine = await reader.ReadLineAsync();
        if (string.IsNullOrWhiteSpace(requestLine))
        {
            throw new InvalidDataException("Empty request.");
        }

        string[] requestParts = requestLine.Split(' ');
        if (requestParts.Length < 2)
        {
            throw new InvalidDataException("Invalid HTTP request line.");
        }

        Dictionary<string, string> headers =
            new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        while (true)
        {
            string line = await reader.ReadLineAsync();
            if (line == null)
            {
                break;
            }

            if (line.Length == 0)
            {
                break;
            }

            int separator = line.IndexOf(':');
            if (separator <= 0)
            {
                continue;
            }

            string key = line.Substring(0, separator).Trim();
            string value = line.Substring(separator + 1).Trim();
            headers[key] = value;
        }

        string body = string.Empty;
        if (headers.TryGetValue("Content-Length", out string lengthText) &&
            int.TryParse(lengthText, out int contentLength) &&
            contentLength > 0)
        {
            char[] buffer = new char[contentLength];
            int offset = 0;
            while (offset < contentLength)
            {
                int read = await reader.ReadAsync(
                    buffer,
                    offset,
                    contentLength - offset
                );
                if (read <= 0)
                {
                    break;
                }

                offset += read;
            }

            body = new string(buffer, 0, offset);
        }

        return new HttpRequest(requestParts[0], requestParts[1], body);
    }

    private static async Task WriteHttpResponse(NetworkStream stream, ApiResponse response)
    {
        string body = response.Body.ToString(Formatting.None);
        byte[] bodyBytes = Encoding.UTF8.GetBytes(body);
        string header =
            $"HTTP/1.1 {response.StatusCode} {HttpStatusText(response.StatusCode)}\r\n" +
            "Content-Type: application/json; charset=utf-8\r\n" +
            $"Content-Length: {bodyBytes.Length}\r\n" +
            "Connection: close\r\n\r\n";
        byte[] headerBytes = Encoding.ASCII.GetBytes(header);
        await stream.WriteAsync(headerBytes, 0, headerBytes.Length);
        await stream.WriteAsync(bodyBytes, 0, bodyBytes.Length);
    }

    private static JToken ParseJsonBody(string body)
    {
        if (string.IsNullOrWhiteSpace(body))
        {
            throw new InvalidDataException("Empty JSON body.");
        }

        return JToken.Parse(body);
    }

    private static bool ContainsStartSignal(JToken token)
    {
        if (token == null)
        {
            return false;
        }

        if (token is JObject obj)
        {
            foreach (JProperty property in obj.Properties())
            {
                if (string.Equals(property.Name, "start", StringComparison.OrdinalIgnoreCase) &&
                    IsTruthyStartValue(property.Value))
                {
                    return true;
                }
            }

            JToken state = obj.GetValue("state", StringComparison.OrdinalIgnoreCase);
            JToken action = obj.GetValue("action", StringComparison.OrdinalIgnoreCase);
            if (state != null &&
                string.Equals(state.ToString(), "start", StringComparison.OrdinalIgnoreCase) &&
                (action == null ||
                 string.Equals(action.ToString(), "activate", StringComparison.OrdinalIgnoreCase)))
            {
                return true;
            }

            foreach (JProperty property in obj.Properties())
            {
                if (ContainsStartSignal(property.Value))
                {
                    return true;
                }
            }
        }

        if (token is JArray array)
        {
            foreach (JToken item in array)
            {
                if (ContainsStartSignal(item))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private static bool IsTruthyStartValue(JToken token)
    {
        if (token.Type == JTokenType.Boolean)
        {
            return token.Value<bool>();
        }

        if (token.Type == JTokenType.Integer)
        {
            return token.Value<int>() != 0;
        }

        string text = token.ToString().Trim();
        return string.Equals(text, "start", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(text, "true", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(text, "1", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(text, "yes", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(text, "on", StringComparison.OrdinalIgnoreCase);
    }

    private static string TryFindStringField(JToken token, string fieldName)
    {
        if (token is JObject obj)
        {
            JToken value = obj.GetValue(fieldName, StringComparison.OrdinalIgnoreCase);
            if (value != null && value.Type == JTokenType.String)
            {
                return value.Value<string>();
            }

            foreach (JProperty property in obj.Properties())
            {
                string childValue = TryFindStringField(property.Value, fieldName);
                if (!string.IsNullOrWhiteSpace(childValue))
                {
                    return childValue;
                }
            }
        }

        if (token is JArray array)
        {
            foreach (JToken item in array)
            {
                string childValue = TryFindStringField(item, fieldName);
                if (!string.IsNullOrWhiteSpace(childValue))
                {
                    return childValue;
                }
            }
        }

        return null;
    }

    private static string NormalizeRoute(string path)
    {
        if (string.IsNullOrWhiteSpace(path))
        {
            return "/";
        }

        int queryIndex = path.IndexOf('?');
        if (queryIndex >= 0)
        {
            path = path.Substring(0, queryIndex);
        }

        if (!path.StartsWith("/"))
        {
            path = "/" + path;
        }

        path = path.TrimEnd('/');
        return string.IsNullOrEmpty(path) ? "/" : path;
    }

    private static IPAddress ResolveListenAddress(string address)
    {
        if (string.IsNullOrWhiteSpace(address) ||
            string.Equals(address, "0.0.0.0", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(address, "*", StringComparison.OrdinalIgnoreCase))
        {
            return IPAddress.Any;
        }

        if (string.Equals(address, "127.0.0.1", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(address, "localhost", StringComparison.OrdinalIgnoreCase))
        {
            return IPAddress.Loopback;
        }

        if (IPAddress.TryParse(address, out IPAddress parsed))
        {
            return parsed;
        }

        Debug.LogWarning($"Unknown activation listen address '{address}', using 0.0.0.0.");
        return IPAddress.Any;
    }

    private static string HttpStatusText(int statusCode)
    {
        return statusCode switch
        {
            200 => "OK",
            400 => "Bad Request",
            404 => "Not Found",
            405 => "Method Not Allowed",
            _ => "OK"
        };
    }

    private readonly struct HttpRequest
    {
        public readonly string Method;
        public readonly string Path;
        public readonly string Body;

        public HttpRequest(string method, string path, string body)
        {
            Method = method;
            Path = path;
            Body = body;
        }
    }

    private readonly struct ApiResponse
    {
        public readonly int StatusCode;
        public readonly JObject Body;

        public ApiResponse(int statusCode, JObject body)
        {
            StatusCode = statusCode;
            Body = body;
        }
    }

    private void ApplyRuntimeConfiguration()
    {
        string[] args = System.Environment.GetCommandLineArgs();

        if (enableFromCommandLine &&
            (HasArgument(args, enableArgument) ||
             ReadBoolEnvironment("GFSX_ROS_BRIDGE")))
        {
            bridgeEnabled = true;
        }

        if (activationServerFromCommandLine &&
            (HasArgument(args, activationServerArgument) ||
             ReadBoolEnvironment("GFSX_ACTIVATION_SERVER")))
        {
            activationServerEnabled = true;
        }

        if (TryReadArgumentValue(args, activationPortArgument, out string activationPortText) ||
            TryReadEnvironment("GFSX_ACTIVATION_PORT", out activationPortText))
        {
            if (int.TryParse(activationPortText, out int activationPort))
            {
                activationListenPort = activationPort;
            }
            else
            {
                Debug.LogWarning(
                    $"Ignoring invalid activation listener port '{activationPortText}'.",
                    this
                );
            }
        }

        waitingForStartJson =
            activationServerEnabled &&
            requireStartJsonBeforePublishing;
        if (waitingForStartJson)
        {
            bridgeEnabled = false;
        }

        if (!configureConnectionFromCommandLine)
        {
            return;
        }

        if (TryReadArgumentValue(args, rosIpArgument, out string rosIp) ||
            TryReadEnvironment("GFSX_ROS_IP", out rosIp))
        {
            runtimeRosIpOverride = rosIp;
        }

        if (TryReadArgumentValue(args, rosPortArgument, out string rosPortText) ||
            TryReadEnvironment("GFSX_ROS_PORT", out rosPortText))
        {
            if (int.TryParse(rosPortText, out int rosPort))
            {
                runtimeRosPortOverride = rosPort;
            }
            else
            {
                Debug.LogWarning(
                    $"Ignoring invalid ROS port override '{rosPortText}'.",
                    this
                );
            }
        }
    }

    private void ApplyConnectionOverride(ROSConnection connection)
    {
        bool hasIpOverride = !string.IsNullOrWhiteSpace(runtimeRosIpOverride);
        bool hasPortOverride = runtimeRosPortOverride > 0;
        if (!hasIpOverride && !hasPortOverride)
        {
            return;
        }

        string ip = hasIpOverride
            ? runtimeRosIpOverride
            : connection.RosIPAddress;
        int port = hasPortOverride
            ? runtimeRosPortOverride
            : connection.RosPort;

        connection.ConnectOnStart = false;
        connection.Disconnect();
        connection.Connect(ip, port);
    }

    private static bool HasArgument(string[] args, string name)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return false;
        }

        for (int i = 0; i < args.Length; i++)
        {
            if (string.Equals(args[i], name, System.StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    private static bool TryReadArgumentValue(
        string[] args,
        string name,
        out string value)
    {
        value = null;
        if (string.IsNullOrWhiteSpace(name))
        {
            return false;
        }

        string equalsPrefix = name + "=";
        for (int i = 0; i < args.Length; i++)
        {
            string arg = args[i];
            if (arg.StartsWith(equalsPrefix, System.StringComparison.OrdinalIgnoreCase))
            {
                value = arg.Substring(equalsPrefix.Length);
                return !string.IsNullOrWhiteSpace(value);
            }

            if (string.Equals(arg, name, System.StringComparison.OrdinalIgnoreCase) &&
                i + 1 < args.Length)
            {
                value = args[i + 1];
                return !string.IsNullOrWhiteSpace(value);
            }
        }

        return false;
    }

    private static bool TryReadEnvironment(string name, out string value)
    {
        value = System.Environment.GetEnvironmentVariable(name);
        return !string.IsNullOrWhiteSpace(value);
    }

    private static bool ReadBoolEnvironment(string name)
    {
        string value = System.Environment.GetEnvironmentVariable(name);
        return string.Equals(value, "1", System.StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "true", System.StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "yes", System.StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "on", System.StringComparison.OrdinalIgnoreCase);
    }
}
