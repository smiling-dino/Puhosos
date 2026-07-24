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
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public sealed class RobotJsonStartServer : MonoBehaviour
{
    [Header("Server")]
    [SerializeField] private bool serverEnabled = false;
    [SerializeField] private bool enableFromCommandLine = true;
    [SerializeField] private string serverArgument = "--activation-server";
    [SerializeField] private string portArgument = "--activation-port";
    [SerializeField] private string listenAddress = "0.0.0.0";
    [SerializeField] private int listenPort = 8765;

    [Header("Routes")]
    [SerializeField] private string healthPath = "/health";
    [SerializeField] private string setTargetPath = "/set_target";
    [SerializeField] private string activatePath = "/activate";

    [Header("ROS Start Signal")]
    [SerializeField] private bool publishStartSignalToRos = true;
    [SerializeField] private string startSignalTopic = "/cmd_activate";
    [SerializeField] private int startSignalValue = 1;

    private static RobotJsonStartServer activeServer;

    private readonly object stateLock = new object();
    private TcpListener listener;
    private CancellationTokenSource cancellation;
    private Task serverTask;
    private ROSConnection ros;
    private bool ownsServerSlot;
    private bool startSignalPublisherRegistered;
    private bool startAccepted;
    private volatile bool startRequested;
    private string lastTargetClass;

    public bool ServerEnabled => serverEnabled;
    public static RobotJsonStartServer ActiveServer => activeServer;

    public bool StartAccepted
    {
        get
        {
            RobotJsonStartServer source = activeServer != null ? activeServer : this;
            lock (source.stateLock)
            {
                return source.startAccepted;
            }
        }
    }

    public string LastTargetClass
    {
        get
        {
            RobotJsonStartServer source = activeServer != null ? activeServer : this;
            lock (source.stateLock)
            {
                return source.lastTargetClass;
            }
        }
    }

    private void Awake()
    {
        ApplyRuntimeConfiguration();
        StartServerIfNeeded();
    }

    private void OnDisable()
    {
        StopServer("RobotJsonStartServer disabled");
    }

    private void OnDestroy()
    {
        StopServer("RobotJsonStartServer destroyed");
    }

    private void OnApplicationQuit()
    {
        StopServer("Unity application quitting");
    }

    public bool ConsumeStartRequest()
    {
        if (!startRequested)
        {
            return false;
        }

        startRequested = false;
        return true;
    }

    private void StartServerIfNeeded()
    {
        if (!serverEnabled)
        {
            return;
        }

        if (activeServer != null && activeServer != this)
        {
            Debug.LogWarning(
                $"JSON start server already runs on '{activeServer.name}'.",
                this
            );
            return;
        }

        if (listenPort <= 0 || listenPort > 65535)
        {
            Debug.LogError($"Ignoring invalid JSON start server port {listenPort}.", this);
            return;
        }

        try
        {
            IPAddress address = ResolveListenAddress(listenAddress);
            listener = new TcpListener(address, listenPort);
            listener.Start();

            cancellation = new CancellationTokenSource();
            serverTask = Task.Run(() => ServerLoop(cancellation.Token));
            activeServer = this;
            ownsServerSlot = true;

            Debug.Log(
                $"Robot JSON start server opened on {listenAddress}:{listenPort}. " +
                $"POST {activatePath} with {{\"start\":true}} to start the robot model.",
                this
            );
        }
        catch (Exception exception)
        {
            listener = null;
            cancellation?.Dispose();
            cancellation = null;
            serverTask = null;
            Debug.LogError(
                $"Failed to open robot JSON start server: {exception.Message}",
                this
            );
        }
    }

    private void StopServer(string reason)
    {
        bool hadServer = listener != null || cancellation != null || ownsServerSlot;

        try
        {
            cancellation?.Cancel();
            listener?.Stop();
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                $"Failed to stop robot JSON start server: {exception.Message}",
                this
            );
        }

        listener = null;
        cancellation?.Dispose();
        cancellation = null;
        serverTask = null;

        if (ownsServerSlot && activeServer == this)
        {
            activeServer = null;
        }

        ownsServerSlot = false;

        if (hadServer && !string.IsNullOrWhiteSpace(reason))
        {
            Debug.Log($"Robot JSON start server stopped: {reason}.", this);
        }
    }

    private async Task ServerLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            TcpClient client = null;
            try
            {
                client = await listener.AcceptTcpClientAsync();
                _ = Task.Run(() => HandleClient(client, token), token);
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (SocketException)
            {
                if (!token.IsCancellationRequested)
                {
                    Debug.LogWarning("Robot JSON start server socket closed unexpectedly.", this);
                }

                break;
            }
            catch (Exception exception)
            {
                client?.Close();
                if (!token.IsCancellationRequested)
                {
                    Debug.LogWarning(
                        $"Robot JSON start server error: {exception.Message}",
                        this
                    );
                }
            }
        }
    }

    private async Task HandleClient(TcpClient client, CancellationToken token)
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
            bool accepted;
            lock (stateLock)
            {
                targetClass = lastTargetClass;
                accepted = startAccepted;
            }

            return new ApiResponse(
                200,
                new JObject
                {
                    ["ok"] = true,
                    ["serverEnabled"] = serverEnabled,
                    ["startAccepted"] = accepted,
                    ["pendingStartRequest"] = startRequested,
                    ["startSignalTopic"] = startSignalTopic,
                    ["startSignalValue"] = startSignalValue,
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
        lock (stateLock)
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

        string className = TryFindStringField(payload, "class_name");
        lock (stateLock)
        {
            startAccepted = true;
            if (!string.IsNullOrWhiteSpace(className))
            {
                lastTargetClass = className;
            }
        }

        startRequested = true;
        PublishStartSignalToRos();
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

    private void PublishStartSignalToRos()
    {
        if (!publishStartSignalToRos || string.IsNullOrWhiteSpace(startSignalTopic))
        {
            return;
        }

        try
        {
            ros ??= ROSConnection.GetOrCreateInstance();
            if (!startSignalPublisherRegistered)
            {
                ros.RegisterPublisher<Int32Msg>(startSignalTopic);
                startSignalPublisherRegistered = true;
            }

            ros.Publish(startSignalTopic, new Int32Msg(startSignalValue));
            Debug.Log(
                $"Robot JSON start server published {startSignalValue} to '{startSignalTopic}'.",
                this
            );
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                $"Failed to publish robot start signal to '{startSignalTopic}': {exception.Message}",
                this
            );
        }
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
            if (line == null || line.Length == 0)
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

        Debug.LogWarning($"Unknown JSON start server listen address '{address}', using 0.0.0.0.");
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

    private void ApplyRuntimeConfiguration()
    {
        string[] args = Environment.GetCommandLineArgs();

        if (enableFromCommandLine &&
            (HasArgument(args, serverArgument) ||
             ReadBoolEnvironment("GFSX_ACTIVATION_SERVER")))
        {
            serverEnabled = true;
        }

        if (TryReadArgumentValue(args, portArgument, out string portText) ||
            TryReadEnvironment("GFSX_ACTIVATION_PORT", out portText))
        {
            if (int.TryParse(portText, out int port))
            {
                listenPort = port;
            }
            else
            {
                Debug.LogWarning(
                    $"Ignoring invalid JSON start server port '{portText}'.",
                    this
                );
            }
        }
    }

    private static bool HasArgument(string[] args, string name)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return false;
        }

        for (int i = 0; i < args.Length; i++)
        {
            if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
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
            if (arg.StartsWith(equalsPrefix, StringComparison.OrdinalIgnoreCase))
            {
                value = arg.Substring(equalsPrefix.Length);
                return !string.IsNullOrWhiteSpace(value);
            }

            if (string.Equals(arg, name, StringComparison.OrdinalIgnoreCase) &&
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
        value = Environment.GetEnvironmentVariable(name);
        return !string.IsNullOrWhiteSpace(value);
    }

    private static bool ReadBoolEnvironment(string name)
    {
        string value = Environment.GetEnvironmentVariable(name);
        return string.Equals(value, "1", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "true", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "yes", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(value, "on", StringComparison.OrdinalIgnoreCase);
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
}
