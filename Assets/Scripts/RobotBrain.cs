using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

[RequireComponent(typeof(TrackController))]
public class RobotBrain : Agent
{
    [Header("Hardware & Controllers")]
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraServo;
    [SerializeField] private VirtualSensors hardwareSensors;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private ArenaController arenaController;

    [Header("Gripper Simulation")]
    [SerializeField] private bool hasBall = false;
    [SerializeField] private Transform gripperLeftClaw;
    [SerializeField] private Transform gripperRightClaw;

    [Header("Training Robustness")]
    [SerializeField] private bool enableDomainRandomization = true;
    [SerializeField] private int requiredHoldSteps = 50;
    [SerializeField] private float holdStepReward = 0.03f;
    [SerializeField] private float terminalSuccessReward = 8.0f;

    [Header("Extended Domain Randomization")]
    [SerializeField] private bool randomizeVisuals = true;
    [SerializeField] private bool randomizeLighting = true;
    [SerializeField] private bool randomizeSensorModels = true;
    [SerializeField] private bool randomizeObstacleScale = true;
    [SerializeField] private Vector2 sensorScaleRange = new Vector2(0.85f, 1.15f);
    [SerializeField] private Vector2 lightIntensityRange = new Vector2(0.65f, 1.35f);
    [SerializeField] private Vector2 obstacleScaleRange = new Vector2(0.75f, 1.2f);

    [Header("ROS Bridge")]
    [SerializeField] private bool enableRosBridge = false;
    [SerializeField] private float rosPublishInterval = 0.05f;
    [SerializeField] private float rosCommandTimeout = 0.5f;
    [SerializeField] private float rosAngularCmdScale = 1.5f;
    [SerializeField] private string rosCmdVelTopic = "/gfsx/cmd_vel";
    [SerializeField] private string rosCameraTopic = "/gfsx/camera_cmd";
    [SerializeField] private string rosGripperTopic = "/gfsx/gripper_cmd";
    [SerializeField] private string rosUltrasoundTopic = "/gfsx/sensors/ultrasound";
    [SerializeField] private string rosLeftIrTopic = "/gfsx/sensors/left_ir";
    [SerializeField] private string rosRightIrTopic = "/gfsx/sensors/right_ir";
    [SerializeField] private string rosGripperIrTopic = "/gfsx/sensors/gripper_ir";
    [SerializeField] private string rosBallVisibleTopic = "/gfsx/ball/visible";
    [SerializeField] private string rosBallAngleTopic = "/gfsx/ball/angle";
    [SerializeField] private string rosBallDistanceTopic = "/gfsx/ball/distance";
    [SerializeField] private string rosHasBallTopic = "/gfsx/state/has_ball";

    private TrackController trackController;
    private Rigidbody rb;
    
    // Переменные для отслеживания состояния в локальных координатах
    private Vector3 startLocalPosition;
    private float previousDistanceToBall = float.MaxValue;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private bool lastObservedBallVisible = false;
    private float lastObservedBallAngle = 0f;
    private float lastObservedBallDistance = 1f;
    private float lastGasAction = 0f;
    private float lastSteerAction = 0f;
    private int holdTicks = 0;
    private int burstDropoutRemaining = 0;
    private int currentActionLatency = 0;
    private readonly Queue<Vector3> actionBuffer = new Queue<Vector3>();

    private bool defaultsCaptured = false;
    private float defaultRobotMass;
    private float defaultMoveSpeed;
    private float defaultTurnSpeed;
    private float defaultTurnK;
    private float defaultMaxPwmStep;
    private bool ballDefaultsCaptured = false;
    private float defaultBallMass = 1f;
    private Vector3 defaultBallScale = Vector3.one;
    private bool sensorDefaultsCaptured = false;
    private float defaultUsMaxDistance = 2f;
    private int defaultUsRayCount = 5;
    private float defaultUsConeAngle = 30f;
    private float defaultIrObstacleDistance = 0.15f;
    private float defaultIrGripperDistance = 0.08f;
    private float defaultYoloMaxDetectionDistance = 2f;
    private float defaultYoloHorizontalFov = 40f;
    private MaterialPropertyBlock randomizationPropertyBlock;
    private static readonly Dictionary<Light, float> defaultLightIntensities = new Dictionary<Light, float>();
    private static readonly Dictionary<Light, Color> defaultLightColors = new Dictionary<Light, Color>();
    private static int lastLightingRandomizationFrame = -1;

    private static bool rosBridgeOwnerClaimed = false;
    private ROSConnection ros;
    private bool rosBridgeInitialized = false;
    private bool ownsRosBridge = false;
    private float rosPublishTimer = 0f;
    private Vector3 rosDriveCommand = Vector3.zero;
    private int rosGripperCommand = 0;
    private float lastRosCommandTime = -999f;

    public override void Initialize()
    {
        trackController = GetComponent<TrackController>();
        rb = GetComponent<Rigidbody>();
        
        // Запоминаем ЛОКАЛЬНУЮ стартовую позицию относительно нашей арены
        startLocalPosition = transform.localPosition;
        CaptureDefaultDynamics();
        CaptureBallDefaults();
        CaptureSensorDefaults();
        InitializeRosBridge();
    }

    private void FixedUpdate()
    {
        PublishRosTelemetryIfNeeded();
    }

    private void OnDestroy()
    {
        if (ownsRosBridge)
        {
            rosBridgeOwnerClaimed = false;
        }
    }

    public override void OnEpisodeBegin()
    {
        if (trackController != null)
        {
            trackController.ResetMotors();
        }

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        if (gripperController != null)
        {
            gripperController.ReleaseBall();
        }

        hasBall = false;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        lastObservedBallVisible = false;
        lastObservedBallAngle = 0f;
        lastObservedBallDistance = 1f;
        lastGasAction = 0f;
        lastSteerAction = 0f;
        holdTicks = 0;
        burstDropoutRemaining = 0;

        if (arenaController == null)
        {
            Debug.LogError("ArenaController не назначен", this);
            return;
        }

        arenaController.ResetArena();

        transform.localRotation = Quaternion.identity;
        startLocalPosition = transform.localPosition;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        bool useTrainingRandomization = ShouldUseTrainingRandomization();
        ApplyEpisodeRandomization(useTrainingRandomization);
        ResetActionLatency(useTrainingRandomization);

        previousDistanceToBall = GetDistanceToBall();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        bool useTrainingRandomization = ShouldUseTrainingRandomization();
        bool isVisible = yoloCamera != null && yoloCamera.IsBallVisible();
        float normAngle = isVisible ? yoloCamera.GetNormalizedHorizontalAngle() : 0f;

        if (burstDropoutRemaining > 0)
        {
            burstDropoutRemaining--;
        }
        else if (useTrainingRandomization && rb != null && rb.angularVelocity.magnitude > 0.5f && Random.value < 0.15f)
        {
            burstDropoutRemaining = Random.Range(5, 16);
        }

        bool yoloDropout = burstDropoutRemaining > 0;
        bool observedVisible = isVisible && !yoloDropout;
        lastObservedBallVisible = observedVisible;
        lastObservedBallAngle = observedVisible ? normAngle : 0f;
        lastObservedBallDistance = observedVisible ? yoloCamera.GetNormalizedDistance() : 1f;

        if (observedVisible)
        {
            lastKnownBallDirection = Mathf.Sign(normAngle);
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        // Заполнение вектора наблюдений (строго 15 значений)
        float ultrasound = hardwareSensors != null ? hardwareSensors.ultrasoundValue : 1f;
        float ultrasoundNoise = useTrainingRandomization ? Random.Range(-0.05f, 0.05f) : 0f;
        sensor.AddObservation(Mathf.Clamp01(ultrasound + ultrasoundNoise));               // 1. УЗ-дальномер с шумом
        sensor.AddObservation(hardwareSensors != null ? hardwareSensors.leftIRObstacle : 0);  // 2. Левый ИК
        sensor.AddObservation(hardwareSensors != null ? hardwareSensors.rightIRObstacle : 0); // 3. Правый ИК
        sensor.AddObservation(hardwareSensors != null && hardwareSensors.isBallInGripper ? 1.0f : 0.0f); // 4. ИК в клешне
        sensor.AddObservation(observedVisible ? normAngle : 0f);                          // 5. Угол до мяча
        sensor.AddObservation(lastObservedBallDistance);                                   // 6. Дистанция до мяча
        sensor.AddObservation(lastKnownBallDirection);                                    // 7. Направление утерянного мяча
        sensor.AddObservation(observedVisible ? 1.0f : 0.0f);                             // 8. Видимость мяча
        
        float servoRotationNorm = cameraServo != null ? cameraServo.localEulerAngles.y / 360f : 0f;
        sensor.AddObservation(servoRotationNorm);                                         // 9. Поворот серво камеры

        sensor.AddObservation(hasBall ? 1.0f : 0.0f);                                     // 10. Статус захвата мяча
        
        // Смещение считаем в локальном пространстве арены
        Vector3 offset = transform.localPosition - startLocalPosition;
        sensor.AddObservation(offset.x);                                                  // 11. Смещение X
        sensor.AddObservation(offset.z);                                                  // 12. Смещение Z
        
        float headingAngle = transform.localEulerAngles.y / 360f;
        sensor.AddObservation(headingAngle);                                              // 13. Угол взгляда робота (локальный)
        sensor.AddObservation(rb != null ? rb.linearVelocity.magnitude : 0f);              // 14. Физическая скорость
        sensor.AddObservation(timeSinceLastDetection);                                    // 15. Время без детекций
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Считывание непрерывных действий с опциональной задержкой команд
        Vector3 delayedActions = GetDelayedContinuousActions(actions);
        float gas = delayedActions.x;
        float steer = delayedActions.y;
        float cameraTurn = delayedActions.z;

        int gripperAction = actions.DiscreteActions[0];
        bool holdingBeforeAction = hasBall || (gripperController != null && gripperController.IsHoldingBall());
        if (holdingBeforeAction && gripperAction != 2)
        {
            gas = 0f;
            steer = 0f;
            cameraTurn = 0f;
        }

        // Применяем движение к моторам гусениц
        if (trackController != null)
        {
            trackController.SetInputs(gas, steer);
        }

        // Поворот камеры
        if (cameraServo != null)
        {
            cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime);
        }

        // 2. Считывание дискретного действия (Клешня)
        ExecuteGripperAction(gripperAction);

        // 3. Расчет наград
        CalculateRewards(gas, steer, gripperAction);

        lastGasAction = gas;
        lastSteerAction = steer;
    }

    private void ExecuteGripperAction(int actionID)
    {
        if (actionID == 1) // Закрыть
        {
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 15, 0);
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, -15, 0);

            // Физический захват мяча через GripperController
            if (hardwareSensors != null && hardwareSensors.isBallInGripper && yoloCamera != null)
            {
                Transform ballTransform = yoloCamera.GetBallTransform();
                if (ballTransform != null && gripperController != null && !gripperController.IsHoldingBall())
                {
                    gripperController.GrabBall(ballTransform.gameObject);
                    hasBall = true;
                }
            }
        }
        else if (actionID == 2) // Открыть
        {
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 0, 0);
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, 0, 0);
            
            if (gripperController != null)
            {
                gripperController.ReleaseBall();
            }
            hasBall = false;
            holdTicks = 0;
        }
    }

    private void CalculateRewards(float gas, float steer, int gripperAction)
    {
        float currentDistance = GetDistanceToBall();
        float distanceDelta = previousDistanceToBall - currentDistance;

        // Небольшая цена времени и энергии, чтобы бесцельная езда не была нейтральной стратегией.
        AddReward(-0.001f);
        AddReward(-(Mathf.Abs(gas) * 0.0005f + Mathf.Abs(steer) * 0.001f));

        float progressReward = 0f;
        if (distanceDelta > 0)
        {
            float multiplier = currentDistance < 0.5f ? 3.0f : 1.5f;
            progressReward = Mathf.Clamp(distanceDelta, 0f, 0.08f) * multiplier;
        }
        else if (distanceDelta < 0)
        {
            progressReward = Mathf.Clamp(distanceDelta, -0.08f, 0f) * 0.5f;
        }
        AddReward(progressReward);
        previousDistanceToBall = currentDistance;

        if (lastObservedBallVisible)
        {
            float centerDiff = Mathf.Abs(lastObservedBallAngle);
            AddReward(0.01f);
            AddReward((1.0f - centerDiff) * 0.025f);
            AddReward((1.0f - lastObservedBallDistance) * 0.01f);
        }
        else
        {
            AddReward(-0.002f);
        }

        if (currentDistance < 1.2f)
        {
            AddReward((1.2f - currentDistance) * 0.01f);
        }

        if (currentDistance < 0.4f)
        {
            AddReward(0.03f);
        }

        float gasJerk = Mathf.Abs(gas - lastGasAction);
        float steerJerk = Mathf.Abs(steer - lastSteerAction);
        AddReward(-(gasJerk + steerJerk) * 0.005f);

        if (hardwareSensors != null && hardwareSensors.isBallInGripper && !hasBall)
        {
            AddReward(0.08f);
        }

        if (gripperAction == 1)
        {
            AddReward(hardwareSensors != null && hardwareSensors.isBallInGripper ? 0.25f : -0.01f);
        }
        else if (gripperAction == 2 && !hasBall)
        {
            AddReward(-0.005f);
        }

        if (hardwareSensors != null)
        {
            float wallPenalty = 0f;
            if (hardwareSensors.ultrasoundValue < 0.25f)
            {
                wallPenalty += Mathf.Lerp(0.0f, 0.04f, 1.0f - hardwareSensors.ultrasoundValue / 0.25f);
            }

            if (hardwareSensors.leftIRObstacle == 1 || hardwareSensors.rightIRObstacle == 1)
            {
                wallPenalty += Mathf.Abs(gas) > 0.05f ? 0.03f : 0.015f;
            }

            AddReward(-wallPenalty);
        }

        if (hasBall || (gripperController != null && gripperController.IsHoldingBall()))
        {
            hasBall = true;
            holdTicks++;
            AddReward(holdStepReward);

            if (holdTicks >= requiredHoldSteps)
            {
                AddReward(terminalSuccessReward);
                Academy.Instance.StatsRecorder.Add("GFSX/Success", 1f, StatAggregationMethod.Sum);
                EndEpisode();
            }
            return;
        }
        holdTicks = 0;

        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add("GFSX/DistanceToBall", currentDistance);
            Academy.Instance.StatsRecorder.Add("GFSX/BallVisible", lastObservedBallVisible ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("GFSX/HoldingBall", hasBall ? 1f : 0f);
        }

        // Проверка падения или опрокидывания
        float tiltX = Mathf.Abs(
            Mathf.DeltaAngle(0f, transform.localEulerAngles.x)
        );

        float tiltZ = Mathf.Abs(
            Mathf.DeltaAngle(0f, transform.localEulerAngles.z)
        );

        if (transform.localPosition.y < -1f || tiltX > 45f || tiltZ > 45f)
        {
            AddReward(-3f);
            Academy.Instance.StatsRecorder.Add("GFSX/FallOrFlip", 1f, StatAggregationMethod.Sum);
            EndEpisode();
            return;
        }
    }

    private float GetDistanceToBall()
    {
        if (yoloCamera == null) return 10f;

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null) return 10f;
        return Vector3.Distance(transform.position, ball.position);
    }

    private bool ShouldUseTrainingRandomization()
    {
        return enableDomainRandomization && Academy.Instance.IsCommunicatorOn;
    }

    private Vector3 GetDelayedContinuousActions(ActionBuffers actions)
    {
        Vector3 currentActions = new Vector3(
            actions.ContinuousActions[0],
            actions.ContinuousActions[1],
            actions.ContinuousActions[2]
        );

        if (!ShouldUseTrainingRandomization() || currentActionLatency <= 0)
        {
            return currentActions;
        }

        actionBuffer.Enqueue(currentActions);
        return actionBuffer.Count > 0 ? actionBuffer.Dequeue() : Vector3.zero;
    }

    private void ResetActionLatency(bool useTrainingRandomization)
    {
        currentActionLatency = useTrainingRandomization ? Random.Range(8, 14) : 0;
        actionBuffer.Clear();

        for (int i = 0; i < currentActionLatency; i++)
        {
            actionBuffer.Enqueue(Vector3.zero);
        }
    }

    private void CaptureDefaultDynamics()
    {
        if (defaultsCaptured)
        {
            return;
        }

        if (rb != null)
        {
            defaultRobotMass = rb.mass;
        }

        if (trackController != null)
        {
            defaultMoveSpeed = trackController.moveSpeed;
            defaultTurnSpeed = trackController.turnSpeed;
            defaultTurnK = trackController.turnK;
            defaultMaxPwmStep = trackController.maxPwmStep;
        }

        defaultsCaptured = true;
    }

    private void CaptureBallDefaults()
    {
        if (ballDefaultsCaptured || yoloCamera == null)
        {
            return;
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null)
        {
            return;
        }

        defaultBallScale = ball.localScale;
        Rigidbody ballRb = ball.GetComponent<Rigidbody>();
        if (ballRb != null)
        {
            defaultBallMass = ballRb.mass;
        }

        ballDefaultsCaptured = true;
    }

    private void CaptureSensorDefaults()
    {
        if (sensorDefaultsCaptured)
        {
            return;
        }

        if (hardwareSensors != null)
        {
            defaultUsMaxDistance = hardwareSensors.usMaxDistance;
            defaultUsRayCount = hardwareSensors.usRayCount;
            defaultUsConeAngle = hardwareSensors.usConeAngle;
            defaultIrObstacleDistance = hardwareSensors.irObstacleDistance;
            defaultIrGripperDistance = hardwareSensors.irGripperDistance;
        }

        if (yoloCamera != null)
        {
            defaultYoloMaxDetectionDistance = yoloCamera.MaxDetectionDistance;
            defaultYoloHorizontalFov = yoloCamera.HorizontalFov;
        }

        sensorDefaultsCaptured = true;
    }

    private void ApplySensorRandomization(bool useTrainingRandomization)
    {
        if (!randomizeSensorModels)
        {
            useTrainingRandomization = false;
        }

        if (hardwareSensors != null)
        {
            hardwareSensors.usMaxDistance = useTrainingRandomization ? RandomScaled(defaultUsMaxDistance) : defaultUsMaxDistance;
            hardwareSensors.usConeAngle = useTrainingRandomization ? RandomScaled(defaultUsConeAngle) : defaultUsConeAngle;
            hardwareSensors.usRayCount = useTrainingRandomization
                ? Mathf.Clamp(defaultUsRayCount + Random.Range(-1, 2), 3, 9)
                : defaultUsRayCount;
            hardwareSensors.irObstacleDistance = useTrainingRandomization
                ? RandomScaled(defaultIrObstacleDistance)
                : defaultIrObstacleDistance;
            hardwareSensors.irGripperDistance = useTrainingRandomization
                ? RandomScaled(defaultIrGripperDistance)
                : defaultIrGripperDistance;
        }

        if (yoloCamera != null)
        {
            float detectionDistance = useTrainingRandomization
                ? RandomScaled(defaultYoloMaxDetectionDistance)
                : defaultYoloMaxDetectionDistance;
            float fov = useTrainingRandomization
                ? RandomScaled(defaultYoloHorizontalFov)
                : defaultYoloHorizontalFov;
            yoloCamera.SetDetectionProfile(detectionDistance, fov);
        }
    }

    private void ApplyObstacleRandomization(bool useTrainingRandomization)
    {
        if (!useTrainingRandomization || !randomizeObstacleScale || arenaController == null)
        {
            return;
        }

        foreach (GameObject obstacle in arenaController.SpawnedObstacles)
        {
            if (obstacle == null)
            {
                continue;
            }

            float scaleX = Random.Range(obstacleScaleRange.x, obstacleScaleRange.y);
            float scaleZ = Random.Range(obstacleScaleRange.x, obstacleScaleRange.y);
            obstacle.transform.localScale = new Vector3(scaleX, 1f, scaleZ);
            obstacle.transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }
    }

    private void ApplyVisualRandomization(bool useTrainingRandomization)
    {
        if (arenaController == null)
        {
            return;
        }

        Renderer[] renderers = arenaController.GetComponentsInChildren<Renderer>(true);
        if (useTrainingRandomization && randomizeVisuals && randomizationPropertyBlock == null)
        {
            randomizationPropertyBlock = new MaterialPropertyBlock();
        }

        foreach (Renderer renderer in renderers)
        {
            if (renderer == null)
            {
                continue;
            }

            if (!useTrainingRandomization || !randomizeVisuals)
            {
                renderer.SetPropertyBlock(null);
                continue;
            }

            Color color = PickRandomRendererColor(renderer);
            randomizationPropertyBlock.Clear();
            randomizationPropertyBlock.SetColor("_BaseColor", color);
            randomizationPropertyBlock.SetColor("_Color", color);
            renderer.SetPropertyBlock(randomizationPropertyBlock);
        }
    }

    private void ApplyLightingRandomization(bool useTrainingRandomization)
    {
        if (!randomizeLighting)
        {
            return;
        }

        if (useTrainingRandomization && lastLightingRandomizationFrame == Time.frameCount)
        {
            return;
        }

        Light[] lights = Object.FindObjectsByType<Light>();
        foreach (Light sceneLight in lights)
        {
            if (sceneLight == null)
            {
                continue;
            }

            if (!defaultLightIntensities.ContainsKey(sceneLight))
            {
                defaultLightIntensities[sceneLight] = sceneLight.intensity;
                defaultLightColors[sceneLight] = sceneLight.color;
            }

            if (!useTrainingRandomization)
            {
                sceneLight.intensity = defaultLightIntensities[sceneLight];
                sceneLight.color = defaultLightColors[sceneLight];
                continue;
            }

            sceneLight.intensity = defaultLightIntensities[sceneLight] * Random.Range(lightIntensityRange.x, lightIntensityRange.y);
            sceneLight.color = Random.ColorHSV(0f, 1f, 0.05f, 0.18f, 0.85f, 1f);
        }

        if (useTrainingRandomization)
        {
            lastLightingRandomizationFrame = Time.frameCount;
        }
    }

    private float RandomScaled(float value)
    {
        float min = Mathf.Min(sensorScaleRange.x, sensorScaleRange.y);
        float max = Mathf.Max(sensorScaleRange.x, sensorScaleRange.y);
        return value * Random.Range(min, max);
    }

    private Color PickRandomRendererColor(Renderer renderer)
    {
        if (HasTagInParents(renderer.transform, "TargetBall"))
        {
            return Random.ColorHSV(0f, 1f, 0.75f, 1f, 0.85f, 1f);
        }

        string objectName = renderer.gameObject.name.ToLowerInvariant();
        if (objectName.Contains("plane") || objectName.Contains("floor"))
        {
            return Random.ColorHSV(0f, 1f, 0.08f, 0.25f, 0.35f, 0.75f);
        }

        if (objectName.Contains("obstacle") || objectName.Contains("cube"))
        {
            return Random.ColorHSV(0f, 1f, 0.25f, 0.65f, 0.45f, 0.95f);
        }

        return Random.ColorHSV(0f, 1f, 0.15f, 0.55f, 0.45f, 0.9f);
    }

    private bool HasTagInParents(Transform current, string tagName)
    {
        while (current != null)
        {
            if (current.CompareTag(tagName))
            {
                return true;
            }

            current = current.parent;
        }

        return false;
    }

    private void ApplyEpisodeRandomization(bool useTrainingRandomization)
    {
        CaptureDefaultDynamics();
        CaptureBallDefaults();
        CaptureSensorDefaults();

        if (rb != null)
        {
            rb.mass = useTrainingRandomization ? Random.Range(1.0f, 4.0f) : defaultRobotMass;
        }

        if (trackController != null)
        {
            trackController.moveSpeed = useTrainingRandomization ? Random.Range(0.3f, 0.7f) : defaultMoveSpeed;
            trackController.turnSpeed = useTrainingRandomization ? Random.Range(80f, 160f) : defaultTurnSpeed;
            trackController.turnK = useTrainingRandomization ? Random.Range(0.22f, 0.38f) : defaultTurnK;
            trackController.maxPwmStep = useTrainingRandomization ? Random.Range(8f, 25f) : defaultMaxPwmStep;
        }

        ApplySensorRandomization(useTrainingRandomization);
        ApplyObstacleRandomization(useTrainingRandomization);
        ApplyVisualRandomization(useTrainingRandomization);
        ApplyLightingRandomization(useTrainingRandomization);

        if (yoloCamera == null)
        {
            return;
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null)
        {
            return;
        }

        ball.localScale = useTrainingRandomization
            ? defaultBallScale * Random.Range(0.8f, 1.2f)
            : defaultBallScale;

        Rigidbody ballRb = ball.GetComponent<Rigidbody>();
        if (ballRb != null)
        {
            ballRb.mass = useTrainingRandomization ? defaultBallMass * Random.Range(0.5f, 2.0f) : defaultBallMass;
        }
    }

    private void InitializeRosBridge()
    {
        if (!enableRosBridge || rosBridgeInitialized || Academy.Instance.IsCommunicatorOn)
        {
            return;
        }

        if (rosBridgeOwnerClaimed)
        {
            Debug.LogWarning("ROS bridge is already owned by another RobotBrain instance. This robot will not register duplicate ROS topics.", this);
            return;
        }

        try
        {
            rosBridgeOwnerClaimed = true;
            ownsRosBridge = true;
            ros = ROSConnection.GetOrCreateInstance();

            ros.RegisterPublisher<Float32Msg>(rosUltrasoundTopic);
            ros.RegisterPublisher<BoolMsg>(rosLeftIrTopic);
            ros.RegisterPublisher<BoolMsg>(rosRightIrTopic);
            ros.RegisterPublisher<BoolMsg>(rosGripperIrTopic);
            ros.RegisterPublisher<BoolMsg>(rosBallVisibleTopic);
            ros.RegisterPublisher<Float32Msg>(rosBallAngleTopic);
            ros.RegisterPublisher<Float32Msg>(rosBallDistanceTopic);
            ros.RegisterPublisher<BoolMsg>(rosHasBallTopic);

            ros.Subscribe<TwistMsg>(rosCmdVelTopic, OnRosCmdVel);
            ros.Subscribe<Float32Msg>(rosCameraTopic, OnRosCameraCommand);
            ros.Subscribe<Int32Msg>(rosGripperTopic, OnRosGripperCommand);

            rosBridgeInitialized = true;
            Debug.Log("ROS bridge initialized for GFS-X robot.", this);
        }
        catch (System.Exception exception)
        {
            rosBridgeOwnerClaimed = false;
            ownsRosBridge = false;
            rosBridgeInitialized = false;
            Debug.LogWarning($"ROS bridge initialization failed: {exception.Message}", this);
        }
    }

    private void OnRosCmdVel(TwistMsg message)
    {
        float maxLinearCmd = trackController != null ? trackController.maxLinearCmd : 0.25f;
        float angularScale = Mathf.Max(0.001f, rosAngularCmdScale);

        rosDriveCommand.x = Mathf.Clamp((float)message.linear.x, -maxLinearCmd, maxLinearCmd);
        rosDriveCommand.y = Mathf.Clamp((float)message.angular.z / angularScale, -1f, 1f);
        lastRosCommandTime = Time.time;
    }

    private void OnRosCameraCommand(Float32Msg message)
    {
        rosDriveCommand.z = Mathf.Clamp(message.data, -1f, 1f);
        lastRosCommandTime = Time.time;
    }

    private void OnRosGripperCommand(Int32Msg message)
    {
        rosGripperCommand = Mathf.Clamp(message.data, 0, 2);
        lastRosCommandTime = Time.time;
    }

    private void PublishRosTelemetryIfNeeded()
    {
        if (!rosBridgeInitialized || ros == null)
        {
            return;
        }

        rosPublishTimer += Time.fixedDeltaTime;
        if (rosPublishInterval > 0f && rosPublishTimer < rosPublishInterval)
        {
            return;
        }
        rosPublishTimer = 0f;

        try
        {
            ros.Publish(rosUltrasoundTopic, new Float32Msg(hardwareSensors != null ? hardwareSensors.ultrasoundValue : 1f));
            ros.Publish(rosLeftIrTopic, new BoolMsg(hardwareSensors != null && hardwareSensors.leftIRObstacle != 0));
            ros.Publish(rosRightIrTopic, new BoolMsg(hardwareSensors != null && hardwareSensors.rightIRObstacle != 0));
            ros.Publish(rosGripperIrTopic, new BoolMsg(hardwareSensors != null && hardwareSensors.isBallInGripper));
            ros.Publish(rosBallVisibleTopic, new BoolMsg(lastObservedBallVisible));
            ros.Publish(rosBallAngleTopic, new Float32Msg(lastObservedBallAngle));
            ros.Publish(rosBallDistanceTopic, new Float32Msg(lastObservedBallDistance));
            ros.Publish(rosHasBallTopic, new BoolMsg(hasBall || (gripperController != null && gripperController.IsHoldingBall())));
        }
        catch (System.Exception exception)
        {
            Debug.LogWarning($"ROS telemetry publish failed: {exception.Message}", this);
        }
    }

    private bool TryApplyRosHeuristic(ActionSegment<float> continuous, ActionSegment<int> discrete)
    {
        if (!rosBridgeInitialized)
        {
            return false;
        }

        bool commandIsFresh = rosCommandTimeout <= 0f || Time.time - lastRosCommandTime <= rosCommandTimeout;
        if (!commandIsFresh)
        {
            return false;
        }

        continuous[0] = rosDriveCommand.x;
        continuous[1] = rosDriveCommand.y;
        continuous[2] = rosDriveCommand.z;
        discrete[0] = rosGripperCommand;
        return true;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions;
        ActionSegment<int> discrete = actionsOut.DiscreteActions;

        continuous[0] = 0f;
        continuous[1] = 0f;
        continuous[2] = 0f;
        discrete[0] = 0;

        if (TryApplyRosHeuristic(continuous, discrete))
        {
            return;
        }

        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            return;
        }

        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed)
            continuous[0] = 1f;
        else if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed)
            continuous[0] = -1f;

        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed)
            continuous[1] = 1f;
        else if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed)
            continuous[1] = -1f;

        if (keyboard.eKey.isPressed)
            continuous[2] = 1f;
        else if (keyboard.qKey.isPressed)
            continuous[2] = -1f;

        if (keyboard.spaceKey.isPressed)
            discrete[0] = 1;
        else if (keyboard.leftShiftKey.isPressed)
            discrete[0] = 2;
    }
}
