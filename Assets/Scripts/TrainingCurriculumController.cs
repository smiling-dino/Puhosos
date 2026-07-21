using System.Reflection;
using Unity.MLAgents;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public sealed class TrainingCurriculumController : MonoBehaviour
{
    private const string StageParameterName = "training_stage";
    private const int CaptureOnlyStage = 0;
    private const int CaptureAndLiftStage = 1;
    private const int FullRouteCleanStage = 2;
    private const int FullRouteP1Stage = 3;
    private const int FullRouteP1P2Stage = 4;

    private const float InstallerScanIntervalSeconds = 0.25f;

    private static readonly BindingFlags PrivateInstance =
        BindingFlags.Instance | BindingFlags.NonPublic;

    private static readonly FieldInfo ArenaControllerField =
        typeof(RobotBrain).GetField("arenaController", PrivateInstance);
    private static readonly FieldInfo GripperControllerField =
        typeof(RobotBrain).GetField("gripperController", PrivateInstance);
    private static readonly FieldInfo EverCapturedBallField =
        typeof(RobotBrain).GetField("everCapturedBall", PrivateInstance);
    private static readonly FieldInfo LiftCompletedField =
        typeof(RobotBrain).GetField("liftCompleted", PrivateInstance);

    private static bool bootstrapCreated;

    private bool installerMode;
    private RobotBrain brain;
    private ArenaController arenaController;
    private GripperController gripperController;
    private int activeStage = -1;
    private int lastStepCount = -1;
    private bool trainingConfigured;
    private bool restartPending;
    private bool terminalIssued;
    private float nextInstallerScanTime;

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
                    CaptureOnlyStage
                )
            ),
            CaptureOnlyStage,
            FullRouteP1P2Stage
        );

        if (!trainingConfigured || requestedStage != activeStage)
        {
            activeStage = requestedStage;
            trainingConfigured = true;
            ApplyStageConfiguration();
            ResetEpisodeTracking();
            restartPending = true;
        }

        if (restartPending)
        {
            restartPending = false;
            brain.EndEpisode();
            return;
        }

        if (lastStepCount < 0 || brain.StepCount < lastStepCount)
        {
            ResetEpisodeTracking();
        }
        lastStepCount = brain.StepCount;

        if (brain.StepCount % 100 == 0)
        {
            academy.StatsRecorder.Add("GFSX/CurriculumStage", activeStage);
        }

        if (terminalIssued)
        {
            return;
        }

        if (activeStage == CaptureOnlyStage
            && ReadBrainBool(EverCapturedBallField)
            && IsHoldingBall())
        {
            CompleteLesson("CaptureLessonSuccess");
            return;
        }

        if (activeStage == CaptureAndLiftStage
            && ReadBrainBool(LiftCompletedField)
            && IsHoldingBall())
        {
            CompleteLesson("LiftLessonSuccess");
        }
    }

    private void ResolveAgentReferences()
    {
        arenaController = ReadBrainReference<ArenaController>(ArenaControllerField);
        gripperController = ReadBrainReference<GripperController>(GripperControllerField);

        if (arenaController == null)
        {
            arenaController = GetComponentInParent<ArenaController>();
        }

        if (gripperController == null)
        {
            gripperController = GetComponentInChildren<GripperController>(true);
        }
    }

    private void ApplyStageConfiguration()
    {
        ResolveAgentReferences();

        bool useP1 = activeStage >= FullRouteP1Stage;
        bool useP2 = activeStage >= FullRouteP1P2Stage;
        bool useFullVisualAndLatencyRandomization = useP2;

        SetBrainField("enableDomainRandomization", useP1 || useP2);
        SetBrainField("enableP1Randomization", useP1);
        SetBrainField("enableP2Randomization", useP2);
        SetBrainField("randomizeVisuals", useFullVisualAndLatencyRandomization);
        SetBrainField("randomizeLighting", useFullVisualAndLatencyRandomization);
        SetBrainField("randomizeSensorModels", useP1 || useP2);
        SetBrainField("randomizeObstacleScale", useP1 || useP2);
        SetBrainField("minActionLatencyDecisions", useP2 ? 1 : 0);
        SetBrainField("maxActionLatencyDecisions", useP2 ? 4 : 1);

        if (arenaController == null)
        {
            return;
        }

        arenaController.numberOfObstacles = activeStage >= FullRouteP1Stage ? 3 : 0;
        arenaController.defaultInitialVisibleTargetProbability = GetInitialVisibilityProbability();

        SetArenaField(
            "randomizedObstacleCountRange",
            activeStage == FullRouteP1Stage
                ? new Vector2Int(1, 3)
                : new Vector2Int(3, 10)
        );
        SetArenaField(
            "randomizeFinalTargetPositionEveryEpisode",
            activeStage >= FullRouteP1Stage
        );
        SetArenaField("useMultipleBallTypes", useP2);
        SetArenaField("useProceduralFloorTextures", useP2);
        SetArenaField("partialOcclusionProbability", useP2 ? 0.3f : 0f);
        SetArenaField(
            "redDistractorCountRange",
            useP2 ? new Vector2Int(0, 3) : Vector2Int.zero
        );
    }

    private float GetInitialVisibilityProbability()
    {
        switch (activeStage)
        {
            case CaptureOnlyStage:
            case CaptureAndLiftStage:
            case FullRouteCleanStage:
                return 1f;
            case FullRouteP1Stage:
                return 0.7f;
            default:
                return 0.2f;
        }
    }

    private bool IsHoldingBall()
    {
        return gripperController != null && gripperController.IsHoldingBall();
    }

    private void CompleteLesson(string metricName)
    {
        terminalIssued = true;
        brain.AddReward(1f);

        Academy.Instance.StatsRecorder.Add(
            $"GFSX/{metricName}",
            1f,
            StatAggregationMethod.Sum
        );
        Academy.Instance.StatsRecorder.Add(
            "GFSX/CurriculumStageSuccess",
            1f,
            StatAggregationMethod.Sum
        );
        Academy.Instance.StatsRecorder.Add(
            $"GFSX/Reward/{metricName}",
            1f,
            StatAggregationMethod.Sum
        );

        brain.EndEpisode();
    }

    private void ResetEpisodeTracking()
    {
        terminalIssued = false;
        lastStepCount = brain != null ? brain.StepCount : -1;
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
