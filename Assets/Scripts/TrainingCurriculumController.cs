using System.Reflection;
using Unity.MLAgents;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public sealed class TrainingCurriculumController : MonoBehaviour
{
    private const string StageParameterName = "training_stage";

    private const int LookAtVisibleBallStage = 0;
    private const int ApproachFarVisibleStage = 1;
    private const int ApproachNearVisibleStage = 2;
    private const int ReachCaptureZoneStage = 3;
    private const int CloseGripperAssistedStage = 4;
    private const int CaptureNearVisibleStage = 5;
    private const int FoldHalfPregraspedStage = 6;
    private const int FoldFullPregraspedStage = 7;
    private const int LiftPregraspedStage = 8;
    private const int CaptureAndLiftNearStage = 9;
    private const int CarryToTargetAssistedStage = 10;
    private const int StopAtTargetAssistedStage = 11;
    private const int FullRouteShortCleanStage = 12;
    private const int FullRouteCleanStage = 13;
    private const int SearchAndFullRouteCleanStage = 14;
    private const int FullRouteP1DynamicsStage = 15;
    private const int FullRouteP1SensorsStage = 16;
    private const int FullRouteP1P2MildStage = 17;
    private const int FullRouteP1P2FullStage = 18;

    private const float InstallerScanIntervalSeconds = 0.25f;

    private static readonly BindingFlags PrivateInstance =
        BindingFlags.Instance | BindingFlags.NonPublic;

    private static readonly FieldInfo ArenaControllerField =
        typeof(RobotBrain).GetField("arenaController", PrivateInstance);
    private static readonly FieldInfo GripperControllerField =
        typeof(RobotBrain).GetField("gripperController", PrivateInstance);
    private static readonly FieldInfo YoloCameraField =
        typeof(RobotBrain).GetField("yoloCamera", PrivateInstance);
    private static readonly FieldInfo EverCapturedBallField =
        typeof(RobotBrain).GetField("everCapturedBall", PrivateInstance);
    private static readonly FieldInfo LiftCompletedField =
        typeof(RobotBrain).GetField("liftCompleted", PrivateInstance);

    private static bool bootstrapCreated;

    private bool installerMode;
    private RobotBrain brain;
    private ArenaController arenaController;
    private GripperController gripperController;
    private SimulatedYoloCamera yoloCamera;
    private Rigidbody robotRigidbody;

    private int activeStage = -1;
    private int lastStepCount = -1;
    private int stableSuccessPhysicsSteps;
    private bool trainingConfigured;
    private bool restartPending;
    private bool episodeObserved;
    private bool episodePrepared;
    private bool terminalIssued;
    private bool suppressFailureForNextReset;
    private float nextInstallerScanTime;
    private float episodeBallStartY;

    private float currentStopDistance = 0.45f;
    private float currentStopTolerance = 0.15f;
    private float currentMaxPlanarSpeed = 0.05f;
    private float currentMaxAngularSpeed = 8f;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void CreateBootstrap()
    {
        if (bootstrapCreated)
        {
            return;
        }

        bootstrapCreated = true;
        GameObject bootstrap = new GameObject("TrainingCurriculumBootstrap");
        DontDestroyOnLoad(bootstrap);
        bootstrap.AddComponent<TrainingCurriculumController>();
    }

    private void Awake()
    {
        brain = GetComponent<RobotBrain>();
        installerMode = brain == null;

        if (!installerMode)
        {
            ResolveAgentReferences();
        }
    }

    private void Update()
    {
        if (!installerMode || Time.unscaledTime < nextInstallerScanTime)
        {
            return;
        }

        nextInstallerScanTime = Time.unscaledTime + InstallerScanIntervalSeconds;
        RobotBrain[] brains = Object.FindObjectsByType<RobotBrain>(FindObjectsSortMode.None);
        foreach (RobotBrain foundBrain in brains)
        {
            if (foundBrain == null
                || foundBrain.TryGetComponent<TrainingCurriculumController>(out _))
            {
                continue;
            }

            foundBrain.gameObject.AddComponent<TrainingCurriculumController>();
        }
    }

    private void FixedUpdate()
    {
        if (installerMode || brain == null || !Academy.IsInitialized)
        {
            return;
        }

        Academy academy = Academy.Instance;
        if (!academy.IsCommunicatorOn)
        {
            return;
        }

        int requestedStage = Mathf.Clamp(
            Mathf.RoundToInt(
                academy.EnvironmentParameters.GetWithDefault(
                    StageParameterName,
                    LookAtVisibleBallStage
                )
            ),
            LookAtVisibleBallStage,
            FullRouteP1P2FullStage
        );

        if (!trainingConfigured || requestedStage != activeStage)
        {
            activeStage = requestedStage;
            trainingConfigured = true;
            ApplyStageConfiguration();
            stableSuccessPhysicsSteps = 0;
            episodePrepared = false;
            terminalIssued = false;
            suppressFailureForNextReset = true;
            restartPending = true;
        }

        if (restartPending)
        {
            restartPending = false;
            brain.EndEpisode();
            return;
        }

        bool episodeRestarted = episodeObserved
            && lastStepCount >= 0
            && brain.StepCount < lastStepCount;

        if (!episodeObserved || episodeRestarted)
        {
            if (episodeObserved
                && !terminalIssued
                && !suppressFailureForNextReset)
            {
                RecordEpisodeResult(false);
            }

            episodeObserved = true;
            episodePrepared = false;
            terminalIssued = false;
            stableSuccessPhysicsSteps = 0;
            suppressFailureForNextReset = false;
        }

        if (!episodePrepared && brain.StepCount <= 1)
        {
            PrepareStageEpisode();
            episodePrepared = true;
        }

        lastStepCount = brain.StepCount;

        if (brain.StepCount % 100 == 0)
        {
            academy.StatsRecorder.Add(
                "GFSX/CurriculumStage",
                activeStage,
                StatAggregationMethod.MostRecent
            );
        }

        if (!terminalIssued)
        {
            EvaluateStageSuccess();
        }
    }

    private void ResolveAgentReferences()
    {
        arenaController = ReadBrainReference<ArenaController>(ArenaControllerField);
        gripperController = ReadBrainReference<GripperController>(GripperControllerField);
        yoloCamera = ReadBrainReference<SimulatedYoloCamera>(YoloCameraField);
        robotRigidbody = brain != null ? brain.GetComponent<Rigidbody>() : null;

        if (arenaController == null)
        {
            arenaController = GetComponentInParent<ArenaController>();
        }

        if (gripperController == null)
        {
            gripperController = GetComponentInChildren<GripperController>(true);
        }

        if (yoloCamera == null)
        {
            yoloCamera = GetComponentInChildren<SimulatedYoloCamera>(true);
        }
    }

    private void ApplyStageConfiguration()
    {
        ResolveAgentReferences();
        ConfigureTaskDifficulty();
        ConfigureRewardProfile();
        ConfigureDomainRandomization();
        brain.MaxStep = GetStageMaxStep();

        if (Academy.IsInitialized
            && Academy.Instance.IsCommunicatorOn
            && gripperController != null)
        {
            Academy.Instance.StatsRecorder.Add(
                "GFSX/ReachableLiftHeight",
                gripperController.ReachableLiftHeightMeters,
                StatAggregationMethod.MostRecent
            );
        }
    }

    private void ConfigureTaskDifficulty()
    {
        int stableGraspSteps = activeStage <= CloseGripperAssistedStage
            ? 3
            : activeStage <= CaptureNearVisibleStage
                ? 5
                : 8;

        float captureRadius = activeStage == CloseGripperAssistedStage
            ? 0.14f
            : activeStage == CaptureNearVisibleStage
                ? 0.12f
                : activeStage == CaptureAndLiftNearStage
                    ? 0.11f
                    : 0.10f;

        float liftHeight = activeStage <= LiftPregraspedStage
            ? 0.03f
            : activeStage <= CaptureAndLiftNearStage
                ? 0.04f
                : activeStage <= FullRouteShortCleanStage
                    ? 0.05f
                    : activeStage == FullRouteCleanStage
                        ? 0.06f
                        : activeStage == SearchAndFullRouteCleanStage
                            ? 0.07f
                            : 0.08f;

        if (gripperController != null && gripperController.ReachableLiftHeightMeters > 0.02f)
        {
            float reachableFraction = activeStage <= LiftPregraspedStage
                ? 0.40f
                : activeStage <= CaptureAndLiftNearStage
                    ? 0.55f
                    : activeStage <= FullRouteShortCleanStage
                        ? 0.70f
                        : activeStage <= FullRouteCleanStage
                            ? 0.80f
                            : 0.90f;

            liftHeight = Mathf.Min(
                liftHeight,
                gripperController.ReachableLiftHeightMeters * reachableFraction
            );
        }

        int stableLiftDecisions = activeStage <= LiftPregraspedStage
            ? 3
            : activeStage <= CaptureAndLiftNearStage
                ? 4
                : activeStage <= FullRouteShortCleanStage
                    ? 5
                    : activeStage <= FullRouteCleanStage
                        ? 6
                        : activeStage <= FullRouteP1SensorsStage
                            ? 8
                            : 10;

        currentStopTolerance = activeStage <= StopAtTargetAssistedStage
            ? 0.25f
            : activeStage == FullRouteShortCleanStage
                ? 0.22f
                : activeStage == FullRouteCleanStage
                    ? 0.18f
                    : 0.15f;

        currentMaxPlanarSpeed = activeStage <= StopAtTargetAssistedStage
            ? 0.10f
            : activeStage == FullRouteShortCleanStage
                ? 0.08f
                : 0.05f;

        currentMaxAngularSpeed = activeStage <= StopAtTargetAssistedStage
            ? 12f
            : activeStage == FullRouteShortCleanStage
                ? 10f
                : 8f;

        int stableStopDecisions = activeStage <= StopAtTargetAssistedStage
            ? 3
            : activeStage == FullRouteShortCleanStage
                ? 5
                : activeStage == FullRouteCleanStage
                    ? 7
                    : 10;

        SetBrainField("requiredStableGraspPhysicsSteps", stableGraspSteps);
        SetBrainField("gripperCaptureRadius", captureRadius);
        SetBrainField("requiredLiftHeightMeters", liftHeight);
        SetBrainField("requiredStableLiftDecisions", stableLiftDecisions);
        SetBrainField("targetStopDistanceMeters", currentStopDistance);
        SetBrainField("targetStopToleranceMeters", currentStopTolerance);
        SetBrainField("maxStopPlanarSpeedMetersPerSecond", currentMaxPlanarSpeed);
        SetBrainField("maxStopAngularSpeedDegreesPerSecond", currentMaxAngularSpeed);
        SetBrainField("requiredStableStopDecisions", stableStopDecisions);
        SetBrainField(
            "maxDistanceForPotentialMeters",
            activeStage <= ApproachNearVisibleStage ? 1.25f : 2.0f
        );
        SetBrainField(
            "maxTargetDistanceForPotentialMeters",
            activeStage <= StopAtTargetAssistedStage ? 1.25f : 2.0f
        );
    }

    private void ConfigureRewardProfile()
    {
        bool elementary = activeStage <= ReachCaptureZoneStage;
        bool manipulation = activeStage >= CloseGripperAssistedStage
            && activeStage <= CaptureAndLiftNearStage;
        bool route = activeStage >= CarryToTargetAssistedStage;
        bool robust = activeStage >= FullRouteP1DynamicsStage;

        SetBrainField("timePenaltyPerDecision", elementary ? 0f : route ? 0.00010f : 0.00005f);
        SetBrainField("idlePenaltyPerDecision", elementary ? 0f : route ? 0.00005f : 0f);
        SetBrainField("distancePotentialWeight", route ? 0.10f : 0.08f);
        SetBrainField("bodyAlignmentPotentialWeight", 0.02f);
        SetBrainField("firstAcquisitionReward", elementary ? 0.01f : 0.02f);
        SetBrainField("captureZoneReward", manipulation || route ? 0.04f : 0f);
        SetBrainField("confirmedGraspReward", manipulation || route ? 0.08f : 0f);
        SetBrainField("foldPotentialWeight", activeStage >= FoldHalfPregraspedStage ? 0.05f : 0f);
        SetBrainField("liftPotentialWeight", activeStage >= LiftPregraspedStage ? 0.10f : 0f);
        SetBrainField("liftCompletedReward", activeStage >= LiftPregraspedStage ? 0.08f : 0f);
        SetBrainField("targetDistancePotentialWeight", route ? 0.15f : 0f);
        SetBrainField("targetAlignmentPotentialWeight", route ? 0.02f : 0f);
        SetBrainField("terminalSuccessReward", 1f);

        SetBrainField("failedGraspPenalty", elementary ? 0f : manipulation ? -0.01f : -0.02f);
        SetBrainField("ballLostBeforeCapturePenalty", elementary ? 0f : robust ? -0.75f : -0.35f);
        SetBrainField("droppedBallPenalty", elementary ? 0f : robust ? -0.35f : -0.15f);
        SetBrainField("fallOrFlipPenalty", -1f);
        SetBrainField("timeoutPenalty", elementary ? 0f : manipulation ? -0.10f : -0.25f);
        SetBrainField("ordinaryCollisionPenalty", robust ? -0.08f : route ? -0.03f : 0f);
        SetBrainField("sustainedCollisionPenaltyPerDecision", robust ? -0.001f : 0f);
        SetBrainField("reverseMotionPenaltyScale", robust ? 0.0002f : 0f);
        SetBrainField("wheelEffortPenaltyScale", route ? 0.00015f : 0.00005f);
        SetBrainField("cameraEffortPenaltyScale", 0.00001f);
        SetBrainField("liftEffortPenaltyScale", 0.00001f);
        SetBrainField("wheelSmoothnessPenaltyScale", robust ? 0.00003f : 0.00001f);
        SetBrainField("auxiliarySmoothnessPenaltyScale", 0.000005f);
        SetBrainField("dangerPenaltyScale", robust ? 0.002f : route ? 0.001f : 0f);
    }

    private void ConfigureDomainRandomization()
    {
        bool useP1 = activeStage >= FullRouteP1DynamicsStage;
        bool useP1Sensors = activeStage >= FullRouteP1SensorsStage;
        bool useP2 = activeStage >= FullRouteP1P2MildStage;
        bool fullP2 = activeStage >= FullRouteP1P2FullStage;

        SetBrainField("enableDomainRandomization", useP1 || useP2);
        SetBrainField("enableP1Randomization", useP1);
        SetBrainField("enableP2Randomization", useP2);
        SetBrainField("randomizeVisuals", useP2);
        SetBrainField("randomizeLighting", useP2);
        SetBrainField("randomizeSensorModels", useP1Sensors);
        SetBrainField("randomizeObstacleScale", useP1Sensors);
        SetBrainField("minActionLatencyDecisions", fullP2 ? 1 : 0);
        SetBrainField("maxActionLatencyDecisions", fullP2 ? 4 : useP2 ? 3 : 1);

        SetBrainField(
            "dynamicsScaleRange",
            fullP2
                ? new Vector2(0.75f, 1.15f)
                : useP2
                    ? new Vector2(0.80f, 1.15f)
                    : useP1Sensors
                        ? new Vector2(0.85f, 1.15f)
                        : new Vector2(0.90f, 1.10f)
        );
        SetBrainField(
            "sensorScaleRange",
            fullP2
                ? new Vector2(0.85f, 1.15f)
                : useP2
                    ? new Vector2(0.90f, 1.10f)
                    : new Vector2(0.95f, 1.05f)
        );
        SetBrainField(
            "commandDecayDecisionRange",
            fullP2 || useP1Sensors ? new Vector2Int(0, 2) : new Vector2Int(0, 1)
        );
        SetBrainField(
            "motorResidualFractionRange",
            fullP2 ? new Vector2(0f, 0.05f) : new Vector2(0f, 0.025f)
        );
        SetBrainField(
            "ultrasoundDropoutProbabilityRange",
            fullP2 ? new Vector2(0.01f, 0.05f) : new Vector2(0f, 0.02f)
        );
        SetBrainField(
            "batteryVoltageScaleRange",
            fullP2 ? new Vector2(0.90f, 1.05f) : new Vector2(0.95f, 1.03f)
        );
        SetBrainField(
            "externalPushIntervalSecondsRange",
            fullP2 ? new Vector2(4f, 12f) : new Vector2(8f, 15f)
        );
        SetBrainField(
            "externalPushImpulseRange",
            fullP2 ? new Vector2(0.02f, 0.08f) : new Vector2(0.01f, 0.04f)
        );

        if (arenaController == null)
        {
            return;
        }

        arenaController.numberOfObstacles = 0;
        arenaController.defaultInitialVisibleTargetProbability = GetInitialVisibilityProbability();
        arenaController.spawnTargetInsideInitialCameraView = true;
        arenaController.targetVisibleMinDistance = activeStage <= ApproachNearVisibleStage ? 0.75f : 1.55f;
        arenaController.targetVisibleMaxDistance = activeStage <= ApproachNearVisibleStage ? 1.20f : 1.85f;

        Vector2Int obstacleRange = Vector2Int.zero;
        if (activeStage == FullRouteP1DynamicsStage)
        {
            obstacleRange = new Vector2Int(1, 1);
        }
        else if (activeStage == FullRouteP1SensorsStage)
        {
            obstacleRange = new Vector2Int(1, 3);
        }
        else if (activeStage == FullRouteP1P2MildStage)
        {
            obstacleRange = new Vector2Int(2, 5);
        }
        else if (activeStage >= FullRouteP1P2FullStage)
        {
            obstacleRange = new Vector2Int(3, 10);
        }

        SetArenaField("randomizedObstacleCountRange", obstacleRange);
        SetArenaField("randomizeFinalTargetPositionEveryEpisode", activeStage >= FullRouteCleanStage);
        SetArenaField("useMultipleBallTypes", fullP2);
        SetArenaField("useProceduralFloorTextures", fullP2);
        SetArenaField("partialOcclusionProbability", fullP2 ? 0.30f : useP2 ? 0.10f : 0f);
        SetArenaField(
            "redDistractorCountRange",
            fullP2
                ? new Vector2Int(0, 3)
                : useP2
                    ? new Vector2Int(0, 1)
                    : Vector2Int.zero
        );
        SetArenaField(
            "maxFloorTiltDegrees",
            fullP2 ? 1.5f : useP2 || useP1Sensors ? 1.0f : useP1 ? 0.5f : 0f
        );
    }

    private float GetInitialVisibilityProbability()
    {
        if (activeStage <= FullRouteCleanStage)
        {
            return 1f;
        }

        switch (activeStage)
        {
            case SearchAndFullRouteCleanStage:
                return 0.50f;
            case FullRouteP1DynamicsStage:
                return 0.80f;
            case FullRouteP1SensorsStage:
                return 0.70f;
            case FullRouteP1P2MildStage:
                return 0.50f;
            default:
                return 0.25f;
        }
    }

    private int GetStageMaxStep()
    {
        switch (activeStage)
        {
            case LookAtVisibleBallStage:
                return 250;
            case ApproachFarVisibleStage:
                return 400;
            case ApproachNearVisibleStage:
                return 600;
            case ReachCaptureZoneStage:
                return 800;
            case CloseGripperAssistedStage:
                return 500;
            case CaptureNearVisibleStage:
                return 1200;
            case FoldHalfPregraspedStage:
                return 300;
            case FoldFullPregraspedStage:
                return 500;
            case LiftPregraspedStage:
                return 700;
            case CaptureAndLiftNearStage:
                return 1600;
            case CarryToTargetAssistedStage:
                return 1000;
            case StopAtTargetAssistedStage:
                return 1400;
            case FullRouteShortCleanStage:
                return 2200;
            case FullRouteCleanStage:
                return 3200;
            case SearchAndFullRouteCleanStage:
                return 3800;
            case FullRouteP1DynamicsStage:
                return 4200;
            case FullRouteP1SensorsStage:
                return 4600;
            default:
                return 5000;
        }
    }

    private void PrepareStageEpisode()
    {
        ResolveAgentReferences();
        stableSuccessPhysicsSteps = 0;

        switch (activeStage)
        {
            case LookAtVisibleBallStage:
                PlaceBallFromHoldPoint(1.10f, Random.Range(-20f, 20f));
                break;
            case ApproachFarVisibleStage:
                PlaceBallFromHoldPoint(1.20f, Random.Range(-15f, 15f));
                break;
            case ApproachNearVisibleStage:
                PlaceBallFromHoldPoint(0.65f, Random.Range(-10f, 10f));
                break;
            case ReachCaptureZoneStage:
                PlaceBallFromHoldPoint(0.30f, Random.Range(-6f, 6f));
                break;
            case CloseGripperAssistedStage:
                PlaceBallFromHoldPoint(0.02f, 0f);
                break;
            case CaptureNearVisibleStage:
                PlaceBallFromHoldPoint(0.35f, Random.Range(-8f, 8f));
                break;
            case FoldHalfPregraspedStage:
            case FoldFullPregraspedStage:
            case LiftPregraspedStage:
                PrepareHeldBall(false);
                break;
            case CaptureAndLiftNearStage:
                PlaceBallFromHoldPoint(0.28f, Random.Range(-10f, 10f));
                break;
            case CarryToTargetAssistedStage:
                PrepareHeldBall(true);
                PlaceFinalTargetFromRobot(1.00f, Random.Range(-20f, 20f));
                break;
            case StopAtTargetAssistedStage:
                PrepareHeldBall(true);
                PlaceFinalTargetFromRobot(0.90f, Random.Range(-15f, 15f));
                break;
            case FullRouteShortCleanStage:
                PlaceBallFromHoldPoint(0.38f, Random.Range(-10f, 10f));
                PlaceFinalTargetFromRobot(1.25f, Random.Range(25f, 50f));
                break;
        }

        if (arenaController != null && arenaController.targetTransform != null)
        {
            episodeBallStartY = arenaController.targetTransform.position.y;
        }
    }

    private void PlaceBallFromHoldPoint(float distanceMeters, float bearingDegrees)
    {
        if (arenaController == null || arenaController.targetTransform == null || brain == null)
        {
            return;
        }

        Transform anchor = gripperController != null && gripperController.HoldPoint != null
            ? gripperController.HoldPoint
            : brain.transform;

        Vector3 forward = brain.transform.TransformDirection(Vector3.right).normalized;
        Vector3 direction = Quaternion.AngleAxis(bearingDegrees, Vector3.up) * forward;
        Vector3 targetPosition = anchor.position + direction * Mathf.Max(0f, distanceMeters);
        targetPosition.y = arenaController.targetTransform.position.y;

        MoveDynamicObject(arenaController.targetTransform, targetPosition, Quaternion.identity);
        arenaController.ResetTargetPhysics();
    }

    private void PlaceFinalTargetFromRobot(float distanceMeters, float bearingDegrees)
    {
        if (arenaController == null || arenaController.FinalTargetTransform == null || brain == null)
        {
            return;
        }

        Vector3 forward = brain.transform.TransformDirection(Vector3.right).normalized;
        Vector3 direction = Quaternion.AngleAxis(bearingDegrees, Vector3.up) * forward;
        Transform finalTarget = arenaController.FinalTargetTransform;
        Vector3 targetPosition = brain.transform.position + direction * Mathf.Max(0f, distanceMeters);
        targetPosition.y = finalTarget.position.y;
        finalTarget.position = targetPosition;
        Physics.SyncTransforms();
    }

    private void PrepareHeldBall(bool alreadyLifted)
    {
        if (arenaController == null
            || arenaController.targetTransform == null
            || gripperController == null)
        {
            return;
        }

        float initialFold = alreadyLifted ? 1f : 0f;
        gripperController.PrepareHeldBallForTraining(
            arenaController.targetTransform.gameObject,
            initialFold
        );

        SetBrainField("hasBall", true);
        SetBrainField("everCapturedBall", true);
        SetBrainField("liftCompleted", alreadyLifted);
        SetBrainField("graspConfirmationPending", false);
        SetBrainField("stableGraspPhysicsStepCount", 0);
        SetBrainField("stableLiftDecisionCount", 0);
        SetBrainField("stableStopDecisionCount", 0);
        SetBrainField("recoveryModeAfterDrop", false);
        SetBrainField("captureZoneAwarded", true);
        SetBrainField("confirmedGraspAwarded", true);
        SetBrainField("liftCompletedAwarded", alreadyLifted);
        SetBrainField("previousFoldPotential", gripperController.LiftNormalized);
        SetBrainField("previousLiftPotential", alreadyLifted ? 1f : 0f);
        SetBrainField("liftStartBallY", arenaController.targetTransform.position.y);

        episodeBallStartY = arenaController.targetTransform.position.y;
    }

    private static void MoveDynamicObject(
        Transform target,
        Vector3 worldPosition,
        Quaternion worldRotation)
    {
        if (target == null)
        {
            return;
        }

        Rigidbody targetRigidbody = target.GetComponent<Rigidbody>();
        if (targetRigidbody == null)
        {
            target.SetPositionAndRotation(worldPosition, worldRotation);
            Physics.SyncTransforms();
            return;
        }

        bool wasKinematic = targetRigidbody.isKinematic;
        targetRigidbody.isKinematic = true;
        targetRigidbody.linearVelocity = Vector3.zero;
        targetRigidbody.angularVelocity = Vector3.zero;
        targetRigidbody.position = worldPosition;
        targetRigidbody.rotation = worldRotation;
        Physics.SyncTransforms();
        targetRigidbody.isKinematic = wasKinematic;
    }

    private void EvaluateStageSuccess()
    {
        float captureDistance = GetCaptureDistance();
        bool holdingBall = IsHoldingBall();
        bool confirmedCapture = holdingBall && ReadBrainBool(EverCapturedBallField);
        bool liftCompleted = holdingBall && ReadBrainBool(LiftCompletedField);

        switch (activeStage)
        {
            case LookAtVisibleBallStage:
                UpdateStableSuccess(
                    yoloCamera != null
                    && yoloCamera.IsBallVisible()
                    && Mathf.Abs(yoloCamera.GetNormalizedHorizontalAngle()) <= 0.15f,
                    10,
                    "LookAtBallSuccess"
                );
                break;
            case ApproachFarVisibleStage:
                UpdateStableSuccess(captureDistance <= 0.80f, 5, "ApproachFarSuccess");
                break;
            case ApproachNearVisibleStage:
                UpdateStableSuccess(captureDistance <= 0.30f, 5, "ApproachNearSuccess");
                break;
            case ReachCaptureZoneStage:
                UpdateStableSuccess(captureDistance <= 0.12f, 5, "CaptureZoneLessonSuccess");
                break;
            case CloseGripperAssistedStage:
            case CaptureNearVisibleStage:
                UpdateStableSuccess(confirmedCapture, 5, "CaptureLessonSuccess");
                break;
            case FoldHalfPregraspedStage:
                UpdateStableSuccess(
                    holdingBall && gripperController != null && gripperController.LiftNormalized >= 0.50f,
                    8,
                    "FoldHalfLessonSuccess"
                );
                break;
            case FoldFullPregraspedStage:
                UpdateStableSuccess(
                    holdingBall && gripperController != null && gripperController.LiftNormalized >= 0.99f,
                    10,
                    "FoldFullLessonSuccess"
                );
                break;
            case LiftPregraspedStage:
                float currentBallY = arenaController != null && arenaController.targetTransform != null
                    ? arenaController.targetTransform.position.y
                    : episodeBallStartY;
                UpdateStableSuccess(
                    holdingBall
                    && gripperController != null
                    && gripperController.LiftNormalized >= 0.99f
                    && currentBallY - episodeBallStartY >= 0.02f,
                    10,
                    "LiftPregraspedLessonSuccess"
                );
                break;
            case CaptureAndLiftNearStage:
                UpdateStableSuccess(liftCompleted, 10, "CaptureAndLiftLessonSuccess");
                break;
            case CarryToTargetAssistedStage:
                UpdateStableSuccess(
                    liftCompleted && GetDistanceToFinalTarget() <= 0.75f,
                    10,
                    "CarryLessonSuccess"
                );
                break;
            default:
                UpdateStableSuccess(
                    liftCompleted && IsWithinStopZone() && IsStopped(),
                    activeStage == StopAtTargetAssistedStage ? 12 : 20,
                    "RouteLessonSuccess"
                );
                break;
        }
    }

    private void UpdateStableSuccess(bool condition, int requiredPhysicsSteps, string metricName)
    {
        stableSuccessPhysicsSteps = condition ? stableSuccessPhysicsSteps + 1 : 0;
        if (stableSuccessPhysicsSteps >= Mathf.Max(1, requiredPhysicsSteps))
        {
            CompleteLesson(metricName);
        }
    }

    private float GetCaptureDistance()
    {
        if (arenaController == null
            || arenaController.targetTransform == null
            || gripperController == null)
        {
            return float.PositiveInfinity;
        }

        return gripperController.GetDistanceToHoldPoint(
            arenaController.targetTransform.gameObject
        );
    }

    private bool IsHoldingBall()
    {
        return gripperController != null && gripperController.IsHoldingBall();
    }

    private float GetDistanceToFinalTarget()
    {
        if (brain == null
            || arenaController == null
            || arenaController.FinalTargetTransform == null)
        {
            return float.PositiveInfinity;
        }

        Vector3 offset = arenaController.FinalTargetTransform.position - brain.transform.position;
        return new Vector2(offset.x, offset.z).magnitude;
    }

    private bool IsWithinStopZone()
    {
        float distance = GetDistanceToFinalTarget();
        return !float.IsNaN(distance)
            && !float.IsInfinity(distance)
            && Mathf.Abs(distance - currentStopDistance) <= currentStopTolerance;
    }

    private bool IsStopped()
    {
        if (robotRigidbody == null)
        {
            return true;
        }

        Vector3 velocity = robotRigidbody.linearVelocity;
        float planarSpeed = new Vector2(velocity.x, velocity.z).magnitude;
        float angularSpeedDegrees = robotRigidbody.angularVelocity.magnitude * Mathf.Rad2Deg;
        return planarSpeed <= currentMaxPlanarSpeed
            && angularSpeedDegrees <= currentMaxAngularSpeed;
    }

    private void CompleteLesson(string metricName)
    {
        if (terminalIssued)
        {
            return;
        }

        terminalIssued = true;
        stableSuccessPhysicsSteps = 0;
        brain.SetReward(1f);
        RecordEpisodeResult(true);

        Academy.Instance.StatsRecorder.Add(
            $"GFSX/Curriculum/{metricName}",
            1f,
            StatAggregationMethod.Sum
        );

        brain.EndEpisode();
    }

    private void RecordEpisodeResult(bool success)
    {
        if (!Academy.IsInitialized || !Academy.Instance.IsCommunicatorOn)
        {
            return;
        }

        float value = success ? 1f : 0f;
        Academy.Instance.StatsRecorder.Add(
            "GFSX/Curriculum/SuccessRate",
            value,
            StatAggregationMethod.Average
        );
        Academy.Instance.StatsRecorder.Add(
            $"GFSX/Curriculum/Stage{activeStage}/SuccessRate",
            value,
            StatAggregationMethod.Average
        );

        if (success)
        {
            Academy.Instance.StatsRecorder.Add(
                "GFSX/Curriculum/SuccessCount",
                1f,
                StatAggregationMethod.Sum
            );
        }
    }

    private bool ReadBrainBool(FieldInfo field)
    {
        return field != null
            && brain != null
            && field.GetValue(brain) is bool value
            && value;
    }

    private T ReadBrainReference<T>(FieldInfo field) where T : class
    {
        return field != null && brain != null
            ? field.GetValue(brain) as T
            : null;
    }

    private void SetBrainField<T>(string fieldName, T value)
    {
        FieldInfo field = typeof(RobotBrain).GetField(fieldName, PrivateInstance);
        field?.SetValue(brain, value);
    }

    private void SetArenaField<T>(string fieldName, T value)
    {
        FieldInfo field = typeof(ArenaController).GetField(fieldName, PrivateInstance);
        field?.SetValue(arenaController, value);
    }
}
