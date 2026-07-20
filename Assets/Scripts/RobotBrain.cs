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
    private enum TaskStage
    {
        Search,
        Approach,
        CaptureReady,
        Holding,
        Lifted
    }

    [Header("Hardware & Controllers")]
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraServo;
    [SerializeField] private VirtualSensors hardwareSensors;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private ArenaController arenaController;

    [Header("Gripper Simulation")]
    [SerializeField] private bool hasBall = false;
    [SerializeField] private float gripperCaptureRadius = 0.08f;

    [Header("Camera Motion")]
    [SerializeField] private float cameraYawLimitDegrees = 90f;
    [SerializeField] private float cameraTurnSpeedDegrees = 50f;

    [Header("Task Completion")]
    [SerializeField] private float requiredLiftHeightMeters = 0.08f;
    [SerializeField] private int requiredStableLiftDecisions = 10;

    [Header("Training Robustness")]
    [SerializeField] private bool enableDomainRandomization = true;
    [SerializeField] private float maxBallDistanceForEpisode = 20f;
    [SerializeField] private float maxBallDropBelowArena = 1.0f;

    [Header("Reward Budget")]
    [SerializeField] private float timePenaltyPerDecision = 0.0005f;
    [SerializeField] private float distancePotentialWeight = 0.30f;
    [SerializeField] private float distancePotentialMaxPerDecision = 0.03f;
    [SerializeField] private float bodyAlignmentPotentialWeight = 0.05f;
    [SerializeField] private float alignmentPotentialMaxPerDecision = 0.01f;
    [SerializeField] private float firstAcquisitionReward = 0.05f;
    [SerializeField] private float captureZoneReward = 0.10f;
    [SerializeField] private float confirmedGraspReward = 0.25f;
    [SerializeField] private float liftPotentialWeight = 0.15f;
    [SerializeField] private float liftPotentialMaxPerDecision = 0.03f;
    [SerializeField] private float terminalSuccessReward = 1.0f;
    [SerializeField] private float ballLostBeforeCapturePenalty = -0.25f;
    [SerializeField] private float droppedBallPenalty = -1.0f;
    [SerializeField] private float fallOrFlipPenalty = -1.0f;

    [Header("Safety & Control Regularization")]
    [SerializeField] private float ordinaryCollisionPenalty = -0.02f;
    [SerializeField] private float collisionPenaltyCooldownSeconds = 0.5f;
    [SerializeField] private float wheelEffortPenaltyScale = 0.00025f;
    [SerializeField] private float cameraEffortPenaltyScale = 0.00002f;
    [SerializeField] private float liftEffortPenaltyScale = 0.00002f;
    [SerializeField] private float wheelSmoothnessPenaltyScale = 0.00005f;
    [SerializeField] private float auxiliarySmoothnessPenaltyScale = 0.00001f;
    [SerializeField] private float dangerPenaltyScale = 0.001f;
    [SerializeField] private float safeUltrasoundDistanceMeters = 0.25f;
    [SerializeField] private float maxDistanceForPotentialMeters = 2.0f;
    [SerializeField] private float maxTimeSinceDetectionSeconds = 5.0f;

    [Header("Extended Domain Randomization")]
    [SerializeField] private bool randomizeVisuals = true;
    [SerializeField] private bool randomizeLighting = true;
    [SerializeField] private bool randomizeSensorModels = true;
    [SerializeField] private bool randomizeObstacleScale = true;
    [SerializeField] private Vector2 sensorScaleRange = new Vector2(0.85f, 1.15f);
    [SerializeField] private Vector2 dynamicsScaleRange = new Vector2(0.75f, 1.15f);
    [SerializeField] private Vector2 lightIntensityRange = new Vector2(0.65f, 1.35f);
    [SerializeField] private Vector2 obstacleScaleRange = new Vector2(0.75f, 1.2f);
    [SerializeField] private int minActionLatencyDecisions = 1;
    [SerializeField] private int maxActionLatencyDecisions = 4;

    [Header("ROS Bridge")]
    [SerializeField] private bool enableRosBridge = false;
    [SerializeField] private float rosPublishInterval = 0.05f;
    [SerializeField] private float rosCommandTimeout = 0.5f;
    [SerializeField] private float rosAngularCmdScale = 1.5f;
    [SerializeField] private string rosCmdVelTopic = "/gfsx/cmd_vel";
    [SerializeField] private string rosCameraTopic = "/gfsx/camera_cmd";
    [SerializeField] private string rosGripperTopic = "/gfsx/gripper_cmd";
    [SerializeField] private string rosLiftTopic = "/gfsx/lift_cmd";
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

    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private bool lastObservedBallVisible = false;
    private float lastObservedBallAngle = 0f;
    private float lastObservedBallDistance = 1f;
    private int burstDropoutRemaining = 0;
    private int currentActionLatency = 0;
    private bool captureAttemptPending = false;
    private readonly Queue<Vector3> actionBuffer = new Queue<Vector3>();

    private TaskStage taskStage = TaskStage.Search;
    private int rewardDecisionPeriod = 5;
    private float cameraYawDegrees = 0f;
    private float previousDistancePotential = 0f;
    private float previousAlignmentPotential = 0f;
    private float previousLiftPotential = 0f;
    private float liftStartBallY = 0f;
    private float previousLeftPwm = 0f;
    private float previousRightPwm = 0f;
    private float previousCameraAction = 0f;
    private float previousLiftAction = 0f;
    private float lastCollisionPenaltyTime = float.NegativeInfinity;
    private bool initialBallVisible = false;
    private bool previousRewardBallVisible = false;
    private bool firstAcquisitionAwarded = false;
    private bool captureZoneAwarded = false;
    private bool confirmedGraspAwarded = false;
    private bool everCapturedBall = false;
    private int stableLiftDecisionCount = 0;

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
    private Quaternion defaultCameraServoLocalRotation = Quaternion.identity;
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
    private int rosLiftCommand = 0;
    private float lastRosCommandTime = -999f;

    public override void Initialize()
    {
        trackController = GetComponent<TrackController>();
        rb = GetComponent<Rigidbody>();
        DecisionRequester decisionRequester = GetComponent<DecisionRequester>();
        if (decisionRequester != null)
        {
            rewardDecisionPeriod = Mathf.Max(1, decisionRequester.DecisionPeriod);
        }

        if (cameraServo != null)
        {
            defaultCameraServoLocalRotation = cameraServo.localRotation;
        }

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
            gripperController.ResetMechanism();
        }

        captureAttemptPending = false;

        hasBall = false;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        lastObservedBallVisible = false;
        lastObservedBallAngle = 0f;
        lastObservedBallDistance = 1f;
        burstDropoutRemaining = 0;
        taskStage = TaskStage.Search;
        cameraYawDegrees = 0f;
        previousDistancePotential = 0f;
        previousAlignmentPotential = 0f;
        previousLiftPotential = 0f;
        liftStartBallY = 0f;
        previousLeftPwm = 0f;
        previousRightPwm = 0f;
        previousCameraAction = 0f;
        previousLiftAction = 0f;
        lastCollisionPenaltyTime = float.NegativeInfinity;
        initialBallVisible = false;
        previousRewardBallVisible = false;
        firstAcquisitionAwarded = false;
        captureZoneAwarded = false;
        confirmedGraspAwarded = false;
        everCapturedBall = false;
        stableLiftDecisionCount = 0;

        if (cameraServo != null)
        {
            cameraServo.localRotation = defaultCameraServoLocalRotation;
        }

        if (arenaController == null)
        {
            Debug.LogError("ArenaController не назначен", this);
            return;
        }

        arenaController.ResetArena();

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        bool useTrainingRandomization = ShouldUseTrainingRandomization();
        ApplyEpisodeRandomization(useTrainingRandomization);
        ResetActionLatency(useTrainingRandomization);

        initialBallVisible = yoloCamera != null && yoloCamera.IsBallVisible();
        previousRewardBallVisible = initialBallVisible;
        lastObservedBallVisible = initialBallVisible;
        lastObservedBallAngle = initialBallVisible ? yoloCamera.GetNormalizedHorizontalAngle() : 0f;
        lastObservedBallDistance = initialBallVisible ? yoloCamera.GetNormalizedDistance() : 1f;
        taskStage = initialBallVisible ? TaskStage.Approach : TaskStage.Search;
        previousDistancePotential = GetDistancePotential();
        previousAlignmentPotential = initialBallVisible ? GetBodyAlignmentPotential() : 0f;

        RecordSpawnDiagnostics(GetDistanceToBall());
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
            timeSinceLastDetection += Time.fixedDeltaTime * Mathf.Max(1, rewardDecisionPeriod);
        }

        // Заполнение вектора наблюдений (строго 15 значений)
        float ultrasound = hardwareSensors != null ? hardwareSensors.ultrasoundValue : 1f;
        float ultrasoundNoise = useTrainingRandomization ? Random.Range(-0.05f, 0.05f) : 0f;
        sensor.AddObservation(Mathf.Clamp01(ultrasound + ultrasoundNoise));               // 1. УЗ-дальномер с шумом
        sensor.AddObservation(hardwareSensors != null ? hardwareSensors.leftIRObstacle : 0);  // 2. Левый ИК
        sensor.AddObservation(hardwareSensors != null ? hardwareSensors.rightIRObstacle : 0); // 3. Правый ИК
        bool ballCaptureReady = IsBallCaptureReady();
        sensor.AddObservation(ballCaptureReady ? 1.0f : 0.0f); // 4. Мяч в зоне клешни
        sensor.AddObservation(observedVisible ? normAngle : 0f);                          // 5. Угол до мяча
        sensor.AddObservation(lastObservedBallDistance);                                   // 6. Дистанция до мяча
        sensor.AddObservation(lastKnownBallDirection);                                    // 7. Направление утерянного мяча
        sensor.AddObservation(observedVisible ? 1.0f : 0.0f);                             // 8. Видимость мяча
        
        float cameraYawNorm = cameraYawLimitDegrees > 0f
            ? Mathf.Clamp(cameraYawDegrees / cameraYawLimitDegrees, -1f, 1f)
            : 0f;
        sensor.AddObservation(cameraYawNorm);                                              // 9. Поворот камеры относительно корпуса
        sensor.AddObservation(IsHoldingBall() ? 1.0f : 0.0f);                             // 10. Подтверждённый захват
        sensor.AddObservation(gripperController != null ? gripperController.LiftNormalized : 0f); // 11. Степень складывания руки
        sensor.AddObservation(trackController != null ? trackController.ForwardPwmNormalized : 0f); // 12. Фактическое движение
        sensor.AddObservation(trackController != null ? trackController.TurnPwmNormalized : 0f);    // 13. Фактический поворот
        sensor.AddObservation(Mathf.Clamp01(
            timeSinceLastDetection / Mathf.Max(0.01f, maxTimeSinceDetectionSeconds)
        ));                                                                                 // 14. Время без детекций
        sensor.AddObservation(
            gripperController != null
                ? gripperController.GripNormalized
                : 0f
        ); // 15. Состояние клешни
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Считывание непрерывных действий с опциональной задержкой команд
        Vector3 delayedActions = GetDelayedContinuousActions(actions);
        float gasNormalized = Mathf.Clamp(delayedActions.x, -1f, 1f);
        float steer = Mathf.Clamp(delayedActions.y, -1f, 1f);
        float cameraTurn = Mathf.Clamp(delayedActions.z, -1f, 1f);

        int gripperAction = actions.DiscreteActions[0];
        int foldAction = actions.DiscreteActions.Length > 1 ? actions.DiscreteActions[1] : 0;
        bool holdingBeforeAction = IsHoldingBall();
        if (holdingBeforeAction)
        {
            gasNormalized = 0f;
            steer = 0f;
            cameraTurn = 0f;
        }

        // Применяем движение к моторам гусениц
        if (trackController != null)
        {
            trackController.SetInputs(gasNormalized * trackController.MaxLinearCmd, steer);
        }

        // Поворот камеры
        if (cameraServo != null)
        {
            cameraYawDegrees = Mathf.Clamp(
                cameraYawDegrees + cameraTurn * cameraTurnSpeedDegrees * Time.fixedDeltaTime,
                -Mathf.Abs(cameraYawLimitDegrees),
                Mathf.Abs(cameraYawLimitDegrees)
            );
            cameraServo.localRotation = defaultCameraServoLocalRotation
                * Quaternion.Euler(0f, cameraYawDegrees, 0f);
        }

        // 2. Дискретные ветви: клешня и согласованное складывание плеча с локтем
        ExecuteGripperAction(gripperAction);
        TryCompletePendingGrasp();
        int appliedFoldAction = IsHoldingBall() ? foldAction : 0;
        if (gripperController != null)
        {
            gripperController.SetLiftAction(appliedFoldAction);
        }

        // 3. Расчет наград
        CalculateRewards(cameraTurn, appliedFoldAction == 1 ? 1f : appliedFoldAction == 2 ? -1f : 0f);
    }

    public override void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
    {
        if (gripperController == null)
        {
            return;
        }

        actionMask.SetActionEnabled(0, 1, !gripperController.IsClosed);
        actionMask.SetActionEnabled(0, 2, gripperController.IsClosed);

        bool canFoldArm = IsHoldingBall() && gripperController.HasLiftActuator;
        actionMask.SetActionEnabled(1, 1, canFoldArm && !gripperController.IsAtLiftTop);
        actionMask.SetActionEnabled(1, 2, canFoldArm && !gripperController.IsAtLiftBottom);
    }

    private void ExecuteGripperAction(int actionID)
    {
        if (gripperController == null)
        {
            return;
        }

        if (actionID == 1 && !gripperController.IsClosed)
        {
            gripperController.SetClosed(true);
            captureAttemptPending = true;
        }
        else if (actionID == 2 && gripperController.IsClosed)
        {
            captureAttemptPending = false;

            gripperController.SetLiftAction(0);
            gripperController.ReleaseBall();
            gripperController.SetClosed(false);

            hasBall = false;
        }
    }

    private void TryCompletePendingGrasp()
    {
        if (!captureAttemptPending
            || gripperController == null
            || !gripperController.IsFullyClosed)
        {
            return;
        }

        captureAttemptPending = false;
        bool captureReady = IsBallCaptureReady();

        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add(
                "GFSX/CaptureAttempts",
                1f,
                StatAggregationMethod.Sum
            );

            Academy.Instance.StatsRecorder.Add(
                "GFSX/CaptureReadyOnAttempt",
                captureReady ? 1f : 0f
            );

            Academy.Instance.StatsRecorder.Add(
                "GFSX/CaptureDistanceOnAttempt",
                GetBallCaptureDistance()
            );
        }

        TryCaptureBall(captureReady);
    }

    private bool TryCaptureBall(bool captureReady)
    {
        if (!captureReady || yoloCamera == null || gripperController == null || gripperController.IsHoldingBall())
        {
            return false;
        }

        Transform ballTransform = yoloCamera.GetBallTransform();
        if (ballTransform == null)
        {
            return false;
        }

        gripperController.GrabBall(ballTransform.gameObject);
        if (!gripperController.IsHoldingBall())
        {
            return false;
        }

        hasBall = true;
        everCapturedBall = true;
        taskStage = TaskStage.Holding;
        liftStartBallY = GetBallYForMetrics();
        previousLiftPotential = 0f;
        stableLiftDecisionCount = 0;

        if (!captureZoneAwarded)
        {
            AddTrackedReward(captureZoneReward, "CaptureZone");
            captureZoneAwarded = true;
        }

        if (!confirmedGraspAwarded)
        {
            AddTrackedReward(confirmedGraspReward, "ConfirmedGrasp");
            confirmedGraspAwarded = true;
        }

        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add("GFSX/CaptureSuccess", 1f, StatAggregationMethod.Sum);
        }
        return true;
    }

    private bool IsHoldingBall()
    {
        return gripperController != null
            ? gripperController.IsHoldingBall()
            : hasBall;
    }

    private bool IsBallCaptureReady()
    {
        if (hasBall || (gripperController != null && gripperController.IsHoldingBall()))
        {
            return true;
        }

        if (hardwareSensors != null)
        {
            return hardwareSensors.isBallInGripper;
        }

        return GetBallCaptureDistance() <= gripperCaptureRadius;
    }

    private float GetBallCaptureDistance()
    {
        if (yoloCamera == null || gripperController == null)
        {
            return float.PositiveInfinity;
        }

        Transform ballTransform = yoloCamera.GetBallTransform();
        if (ballTransform == null)
        {
            return float.PositiveInfinity;
        }

        return gripperController.GetDistanceToHoldPoint(ballTransform.gameObject);
    }

    private void CalculateRewards(float cameraAction, float liftAction)
    {
        float currentDistance = GetDistanceToBall();
        bool holdingBall = IsHoldingBall();
        hasBall = holdingBall;

        if (HasRobotFailed())
        {
            SetTerminalReward(fallOrFlipPenalty, "FallOrFlip");
            return;
        }

        if (everCapturedBall && !holdingBall)
        {
            SetTerminalReward(droppedBallPenalty, "DroppedAfterCapture");
            return;
        }

        if (!everCapturedBall && IsBallLost(currentDistance))
        {
            RecordBallLostMetrics(currentDistance);
            SetTerminalReward(ballLostBeforeCapturePenalty, "BallLostBeforeCapture");
            return;
        }

        if (!IsRewardDecisionStep())
        {
            return;
        }

        AddTrackedReward(-timePenaltyPerDecision, "Time");
        ApplyControlRegularization(cameraAction, liftAction);

        if (holdingBall)
        {
            ApplyLiftRewards();
        }
        else
        {
            ApplyApproachRewards();
        }

        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add("GFSX/DistanceToBall", currentDistance);
            Academy.Instance.StatsRecorder.Add("GFSX/BallVisible", lastObservedBallVisible ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("GFSX/HoldingBall", holdingBall ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("GFSX/CaptureReady", IsBallCaptureReady() ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("GFSX/CaptureDistance", GetBallCaptureDistance());
            Academy.Instance.StatsRecorder.Add("GFSX/LiftHeight", GetCurrentBallLiftHeight());
            Academy.Instance.StatsRecorder.Add(
                "GFSX/ArmFoldNormalized",
                gripperController != null ? gripperController.LiftNormalized : 0f
            );
            Academy.Instance.StatsRecorder.Add("GFSX/TaskStage", (float)taskStage);
        }
    }

    private void ApplyApproachRewards()
    {
        float distancePotential = GetDistancePotential();
        float distanceReward = Mathf.Clamp(
            distancePotentialWeight * (distancePotential - previousDistancePotential),
            -distancePotentialMaxPerDecision,
            distancePotentialMaxPerDecision
        );
        AddTrackedReward(distanceReward, "DistancePotential");
        previousDistancePotential = distancePotential;

        if (!initialBallVisible
            && !firstAcquisitionAwarded
            && lastObservedBallVisible
            && !previousRewardBallVisible)
        {
            AddTrackedReward(firstAcquisitionReward, "FirstAcquisition");
            firstAcquisitionAwarded = true;
        }

        bool captureReady = IsBallCaptureReady();
        if (lastObservedBallVisible)
        {
            taskStage = captureReady ? TaskStage.CaptureReady : TaskStage.Approach;
            float alignmentPotential = GetBodyAlignmentPotential();
            if (previousRewardBallVisible)
            {
                float alignmentReward = Mathf.Clamp(
                    bodyAlignmentPotentialWeight * (alignmentPotential - previousAlignmentPotential),
                    -alignmentPotentialMaxPerDecision,
                    alignmentPotentialMaxPerDecision
                );
                AddTrackedReward(alignmentReward, "BodyAlignmentPotential");
            }

            previousAlignmentPotential = alignmentPotential;
        }
        else
        {
            taskStage = captureReady ? TaskStage.CaptureReady : TaskStage.Search;
            previousAlignmentPotential = 0f;
        }

        previousRewardBallVisible = lastObservedBallVisible;

        if (captureReady && !captureZoneAwarded)
        {
            AddTrackedReward(captureZoneReward, "CaptureZone");
            captureZoneAwarded = true;
        }
    }

    private void ApplyLiftRewards()
    {
        taskStage = TaskStage.Holding;
        float ballHeightPotential =
            requiredLiftHeightMeters > 0f
                ? Mathf.Clamp01(
                    GetCurrentBallLiftHeight()
                    / requiredLiftHeightMeters
                )
                : 1f;

        float actuatorPotential =
            gripperController != null
                ? gripperController.LiftNormalized
                : 0f;

        // Успех возможен только при фактическом подъёме мяча и полном складывании в «Г».
        float liftPotential = Mathf.Min(
            ballHeightPotential,
            actuatorPotential
        );
        float liftReward = Mathf.Clamp(
            liftPotentialWeight * (liftPotential - previousLiftPotential),
            -liftPotentialMaxPerDecision,
            liftPotentialMaxPerDecision
        );
        AddTrackedReward(liftReward, "LiftPotential");
        previousLiftPotential = liftPotential;

        if (liftPotential >= 1f)
        {
            stableLiftDecisionCount++;
        }
        else
        {
            stableLiftDecisionCount = 0;
        }

        if (stableLiftDecisionCount < Mathf.Max(1, requiredStableLiftDecisions))
        {
            return;
        }

        taskStage = TaskStage.Lifted;
        SetTerminalReward(terminalSuccessReward, "Success");
    }

    private void ApplyControlRegularization(float cameraAction, float liftAction)
    {
        float leftPwm = trackController != null ? trackController.LeftPwmNormalized : 0f;
        float rightPwm = trackController != null ? trackController.RightPwmNormalized : 0f;

        float effortPenalty = wheelEffortPenaltyScale * (leftPwm * leftPwm + rightPwm * rightPwm);
        if (lastObservedBallVisible)
        {
            effortPenalty += cameraEffortPenaltyScale * cameraAction * cameraAction;
        }
        effortPenalty += liftEffortPenaltyScale * liftAction * liftAction;
        AddTrackedReward(-effortPenalty, "ControlEffort");

        float smoothnessPenalty = wheelSmoothnessPenaltyScale
            * (Mathf.Abs(leftPwm - previousLeftPwm) + Mathf.Abs(rightPwm - previousRightPwm));
        smoothnessPenalty += auxiliarySmoothnessPenaltyScale
            * (Mathf.Abs(cameraAction - previousCameraAction) + Mathf.Abs(liftAction - previousLiftAction));
        AddTrackedReward(-smoothnessPenalty, "ControlSmoothness");

        if (hardwareSensors != null && safeUltrasoundDistanceMeters > 0f && trackController != null)
        {
            float danger = Mathf.Clamp01(
                (safeUltrasoundDistanceMeters - hardwareSensors.UltrasoundDistanceMeters)
                / safeUltrasoundDistanceMeters
            );
            float movingTowardObstacle = Mathf.Max(0f, trackController.ForwardPwmNormalized);
            AddTrackedReward(-dangerPenaltyScale * danger * movingTowardObstacle, "ApproachingObstacle");
        }

        previousLeftPwm = leftPwm;
        previousRightPwm = rightPwm;
        previousCameraAction = cameraAction;
        previousLiftAction = liftAction;
    }

    private bool IsRewardDecisionStep()
    {
        return StepCount <= 0 || StepCount % Mathf.Max(1, rewardDecisionPeriod) == 0;
    }

    private float GetDistancePotential()
    {
        float distance = GetBallCaptureDistance();
        if (float.IsNaN(distance) || float.IsInfinity(distance))
        {
            return 0f;
        }

        float maxDistance = Mathf.Max(gripperCaptureRadius + 0.01f, maxDistanceForPotentialMeters);
        return 1f - Mathf.Clamp01(
            (distance - gripperCaptureRadius) / (maxDistance - gripperCaptureRadius)
        );
    }

    private float GetBodyAlignmentPotential()
    {
        if (!lastObservedBallVisible || yoloCamera == null)
        {
            return 0f;
        }

        float bodyBearing = Mathf.DeltaAngle(
            0f,
            cameraYawDegrees + yoloCamera.GetSignedHorizontalAngleToBall()
        );
        return 1f - Mathf.Clamp01(Mathf.Abs(bodyBearing) / 180f);
    }

    private float GetCurrentBallLiftHeight()
    {
        return everCapturedBall
            ? Mathf.Max(0f, GetBallYForMetrics() - liftStartBallY)
            : 0f;
    }

    private bool HasRobotFailed()
    {
        float tiltX = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.x));
        float tiltZ = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.z));
        return transform.localPosition.y < -1f || tiltX > 45f || tiltZ > 45f;
    }

    private void RecordBallLostMetrics(float currentDistance)
    {
        if (!Academy.Instance.IsCommunicatorOn)
        {
            return;
        }

        Academy.Instance.StatsRecorder.Add("GFSX/BallLost", 1f, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("GFSX/BallLostDistance", GetSafeMetricValue(currentDistance));
        Academy.Instance.StatsRecorder.Add("GFSX/BallLostY", GetBallYForMetrics());
        Academy.Instance.StatsRecorder.Add("GFSX/BallLostSpeed", GetBallSpeedForMetrics());
    }

    private void AddTrackedReward(float reward, string statName)
    {
        if (Mathf.Approximately(reward, 0f))
        {
            return;
        }

        AddReward(reward);

        if (Academy.Instance.IsCommunicatorOn && !string.IsNullOrEmpty(statName))
        {
            Academy.Instance.StatsRecorder.Add($"GFSX/Reward/{statName}", reward, StatAggregationMethod.Sum);
        }
    }

    private void SetTerminalReward(float reward, string statName)
    {
        SetReward(Mathf.Clamp(reward, -1f, 1f));
        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add($"GFSX/Reward/{statName}", reward, StatAggregationMethod.Sum);
            Academy.Instance.StatsRecorder.Add($"GFSX/{statName}", 1f, StatAggregationMethod.Sum);
        }
        EndEpisode();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision == null
            || collision.transform == null
            || HasTagInParents(collision.transform, "TargetBall")
            || Time.time - lastCollisionPenaltyTime < collisionPenaltyCooldownSeconds)
        {
            return;
        }

        bool hasSideContact = false;
        foreach (ContactPoint contact in collision.contacts)
        {
            if (Mathf.Abs(Vector3.Dot(contact.normal.normalized, Vector3.up)) < 0.7f)
            {
                hasSideContact = true;
                break;
            }
        }

        if (!hasSideContact)
        {
            return;
        }

        lastCollisionPenaltyTime = Time.time;
        AddTrackedReward(ordinaryCollisionPenalty, "Collision");
        if (Academy.Instance.IsCommunicatorOn)
        {
            Academy.Instance.StatsRecorder.Add("GFSX/CollisionCount", 1f, StatAggregationMethod.Sum);
        }
    }

    private float GetDistanceToBall()
    {
        if (yoloCamera == null) return 10f;

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null) return 10f;
        return Vector3.Distance(transform.position, ball.position);
    }

    private bool IsBallLost(float currentDistance)
    {
        if (yoloCamera == null || arenaController == null)
        {
            return false;
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null)
        {
            return false;
        }

        if (!IsFinite(ball.position) || !IsFinite(transform.position) || float.IsNaN(currentDistance) || float.IsInfinity(currentDistance))
        {
            return true;
        }

        float minY = arenaController.transform.position.y - maxBallDropBelowArena;
        return ball.position.y < minY || currentDistance > maxBallDistanceForEpisode;
    }

    private bool IsFinite(Vector3 value)
    {
        return !float.IsNaN(value.x) && !float.IsInfinity(value.x)
            && !float.IsNaN(value.y) && !float.IsInfinity(value.y)
            && !float.IsNaN(value.z) && !float.IsInfinity(value.z);
    }

    private float GetSafeMetricValue(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value)
            ? maxBallDistanceForEpisode + 1f
            : value;
    }

    private float GetBallYForMetrics()
    {
        if (yoloCamera == null)
        {
            return 0f;
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null || !IsFinite(ball.position))
        {
            return 0f;
        }

        return ball.position.y;
    }

    private float GetBallSpeedForMetrics()
    {
        if (yoloCamera == null)
        {
            return 0f;
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null)
        {
            return 0f;
        }

        Rigidbody ballRb = ball.GetComponent<Rigidbody>();
        if (ballRb == null || !IsFinite(ballRb.linearVelocity))
        {
            return 0f;
        }

        return ballRb.linearVelocity.magnitude;
    }

    private bool ShouldUseTrainingRandomization()
    {
        return enableDomainRandomization && Academy.Instance.IsCommunicatorOn;
    }

    private void RecordSpawnDiagnostics(float initialDistanceToBall)
    {
        if (!Academy.Instance.IsCommunicatorOn || arenaController == null)
        {
            return;
        }

        Academy.Instance.StatsRecorder.Add("GFSX/SpawnRejectedObstacles", arenaController.LastRejectedObstacleSpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnRejectedTargets", arenaController.LastRejectedTargetSpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnRejectedTargetVisibility", arenaController.LastRejectedTargetVisibilitySpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnObstacleFallbacks", arenaController.LastObstacleFallbackSpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnTargetFallbacks", arenaController.LastTargetFallbackSpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnVisibleTargetFallbacks", arenaController.LastVisibleTargetFallbackSpawns);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnRequestedVisibleTarget", arenaController.LastTargetRequestedVisible ? 1f : 0f);

        float initialCameraDistanceToBall = yoloCamera != null ? yoloCamera.GetDistanceToBall() : initialDistanceToBall;
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnInitialDistanceToBall", initialDistanceToBall);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnInitialCameraDistanceToBall", initialCameraDistanceToBall);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnInitialBallInRange", yoloCamera != null && initialCameraDistanceToBall <= yoloCamera.MaxDetectionDistance ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnInitialBallHorizontalAngle", yoloCamera != null ? yoloCamera.GetHorizontalAngleToBall() : 180f);
        Academy.Instance.StatsRecorder.Add("GFSX/SpawnInitialBallVisible", yoloCamera != null && yoloCamera.IsBallVisible() ? 1f : 0f);
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
        int minLatency = Mathf.Max(0, minActionLatencyDecisions);
        int maxLatencyExclusive = Mathf.Max(minLatency + 1, maxActionLatencyDecisions);
        currentActionLatency = useTrainingRandomization ? Random.Range(minLatency, maxLatencyExclusive) : 0;
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
            yoloCamera.SetDetectionProfile(defaultYoloMaxDetectionDistance, defaultYoloHorizontalFov);
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
        return RandomScaled(value, sensorScaleRange);
    }

    private float RandomScaled(float value, Vector2 range)
    {
        float min = Mathf.Min(range.x, range.y);
        float max = Mathf.Max(range.x, range.y);
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
            rb.mass = useTrainingRandomization ? defaultRobotMass * Random.Range(0.75f, 1.25f) : defaultRobotMass;
        }

        if (trackController != null)
        {
            trackController.moveSpeed = useTrainingRandomization ? RandomScaled(defaultMoveSpeed, dynamicsScaleRange) : defaultMoveSpeed;
            trackController.turnSpeed = useTrainingRandomization ? RandomScaled(defaultTurnSpeed, dynamicsScaleRange) : defaultTurnSpeed;
            trackController.turnK = useTrainingRandomization ? defaultTurnK * Random.Range(0.85f, 1.15f) : defaultTurnK;
            trackController.maxPwmStep = useTrainingRandomization ? defaultMaxPwmStep * Random.Range(0.75f, 1.25f) : defaultMaxPwmStep;
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

        if (arenaController != null)
        {
            arenaController.ResetTargetPhysics();
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
            ros.Subscribe<Int32Msg>(rosLiftTopic, OnRosLiftCommand);

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

        rosDriveCommand.x = Mathf.Clamp((float)message.linear.x / Mathf.Max(0.001f, maxLinearCmd), -1f, 1f);
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

    private void OnRosLiftCommand(Int32Msg message)
    {
        rosLiftCommand = Mathf.Clamp(message.data, 0, 2);
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
            ros.Publish(rosGripperIrTopic, new BoolMsg(IsBallCaptureReady()));
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
        if (discrete.Length > 1)
        {
            discrete[1] = rosLiftCommand;
        }
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
        if (discrete.Length > 1)
        {
            discrete[1] = 0;
        }

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

        if (discrete.Length > 1)
        {
            if (keyboard.rKey.isPressed)
                discrete[1] = 1;
            else if (keyboard.fKey.isPressed)
                discrete[1] = 2;
        }
    }
}