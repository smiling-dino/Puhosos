using Unity.MLAgents;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

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
    [SerializeField] private bool disableWhileTrainerConnected = true;
    [SerializeField] private bool enforceSinglePublisher = true;

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
    [SerializeField] private float maxLinearSpeedMetersPerSecond = 0.25f;
    [Min(0.001f)]
    [SerializeField] private float maxAngularSpeedRadiansPerSecond = 1.5f;
    [Min(0.001f)]
    [SerializeField] private float cameraTurnSpeedDegreesPerSecond = 50f;

    [Header("Filtering")]
    [Range(0.01f, 1f)]
    [SerializeField] private float emaAlpha = 0.8f;
    [Range(0f, 0.25f)]
    [SerializeField] private float commandDeadzone = 0.01f;
    [Min(0f)]
    [SerializeField] private float minimumPublishIntervalSeconds = 0.05f;

    [Header("Fail-safe")]
    [Min(0.05f)]
    [SerializeField] private float policyWatchdogTimeoutSeconds = 0.5f;
    [SerializeField] private bool publishInitialGripperState = true;

    private static ROSBridge activePublisher;

    private ROSConnection ros;
    private bool initialized;
    private bool ownsPublisherSlot;
    private bool watchdogLatched;
    private bool gripperStateInitialized;

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
    private float lastCameraSampleRealtime;

    private void Awake()
    {
        ResolveReferences();
        CaptureInitialState();
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
        Shutdown("ROSBridge disabled");
    }

    private void OnDestroy()
    {
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
    }

    [ContextMenu("Publish emergency stop")]
    public void PublishEmergencyStop()
    {
        PublishEmergencyStop("manual request");
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

            Debug.Log("ROSBridge initialized for GFS-X inference.", this);
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
        ros.Publish(
            gripperCommandTopic,
            new Int32Msg(closed ? 1 : 2)
        );
        lastClosedState = closed;
        gripperStateInitialized = true;
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

        PublishLift(command);
    }

    private void PublishLift(int command)
    {
        command = Mathf.Clamp(command, 0, 2);
        if (command == lastLiftCommand)
        {
            return;
        }

        ros.Publish(liftCommandTopic, new Int32Msg(command));
        lastLiftCommand = command;
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
}
