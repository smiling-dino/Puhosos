using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;
using Unity.MLAgents.Policies;

[RequireComponent(typeof(Rigidbody))]
public class RobotAgent3 : Agent


{
    [Header("=== ROS Integration ===")]
    public ROSBridge rosBridge;
    private BehaviorParameters behaviorParameters;
    
    [Header("=== Компоненты робота ===")]
    public TrackController trackController;
    public GripperController gripperController;
    public VirtualSensors virtualSensors;
    public SimulatedYoloCamera yoloCamera;
    public ArenaManager arenaManager; 

    [Header("=== Настройки Целей ===")]
    public string ballTag = "TargetBall";
    public string cubeTag = "TargetCube";
    [Tooltip("Дистанция до стенки куба по осям XZ, при которой сброс мяча считается успешным")]
    public float victoryDistance = 0.4f;

    [Header("=== Настройки Наград и Штрафов ===")]
    public float rewardGoalReached = 10.0f;
    public float rewardBallGrabedd = 3.0f;       
    public float rewardCentering = 1.0f;        
    public float rewardDistanceMultiplier = 0.5f; 
    public float rewardHoldBallStep = 0.002f;     
    
    [Space(10)]
    public float penaltyCrash = -2.0f;            
    public float penaltyDropBall = -2.0f;         
    public float penaltyJerkMultiplier = 0.005f;  
    public float penaltyUltrasound = -0.02f;      
    public float penaltyIRObstacle = -0.01f;      
    public float rewardTimePenalty = -0.002f;       
    
    [Space(10)]
    public bool penalizeEmptyGrab = true;
    public float penaltyEmptyGrab = -0.01f;       

    private Rigidbody rb;
    private GameObject targetBall;
    private GameObject targetCube; 

    // --- Состояния для вычислений ИИ ---
    private Vector3 startPosition;
    private Quaternion startRotation;
    
    private float previousDistanceToTarget;
    private float previousTargetQuality; 
    private bool wasTargetVisibleLastFrame = false; 
    
    private float prevGas = 0f;
    private float prevSteer = 0f;
    private float timeSinceLastDetection = 0f;
    private float lastKnownTargetDirection = 0f; 
    
    private bool phaseTwoActive = false; 

    // --- Статистика для TensorBoard ---
    private bool isEpisodeResolved = false;
    private bool hasGrabbedBall = false; // НОВОЕ: Флаг захвата мяча в текущем эпизоде

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.localPosition;
        startRotation = transform.localRotation;

        behaviorParameters = GetComponent<BehaviorParameters>();
    }

    public override void OnEpisodeBegin()
    {
        // НОВОЕ: Сброс статистики
        isEpisodeResolved = false;
        hasGrabbedBall = false; 
        phaseTwoActive = false;

        if (gripperController != null && gripperController.IsHoldingBall())
            gripperController.ReleaseBall();
        
        if (trackController != null)
            trackController.ResetMotors();

        prevGas = 0f;
        prevSteer = 0f;
        timeSinceLastDetection = 0f;
        lastKnownTargetDirection = 0f;
        wasTargetVisibleLastFrame = false;
        previousTargetQuality = 0f;
        
        rb.interpolation = RigidbodyInterpolation.None; 
        transform.localPosition = startPosition; 
        transform.localRotation = startRotation; 
        rb.position = transform.position; 
        rb.rotation = transform.rotation;        
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        if (arenaManager != null)
        {
            arenaManager.ResetArena();
            targetBall = arenaManager.targetBall;
            targetCube = arenaManager.targetCube; 
        }

        UpdateTargetMetrics(true); 
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        YoloTargetInfo ballInfo = yoloCamera != null ? yoloCamera.GetTarget(ballTag) : null;
        YoloTargetInfo cubeInfo = yoloCamera != null ? yoloCamera.GetTarget(cubeTag) : null;

        bool hasBallInSensors = virtualSensors != null && virtualSensors.isBallInGripper;
        YoloTargetInfo currentTargetInfo = hasBallInSensors ? cubeInfo : ballInfo;
        
        if (currentTargetInfo != null && currentTargetInfo.isVisible)
        {
            timeSinceLastDetection = 0f;
            lastKnownTargetDirection = currentTargetInfo.horizontalOffset; 
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        sensor.AddObservation(virtualSensors != null ? virtualSensors.ultrasoundValue : 1f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.leftIRObstacle : 0);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.rightIRObstacle : 0);
        sensor.AddObservation(hasBallInSensors ? 1f : 0f); 

        sensor.AddObservation(ballInfo != null && ballInfo.isVisible ? 1f : 0f);
        sensor.AddObservation(ballInfo != null ? ballInfo.horizontalOffset : 0f);
        sensor.AddObservation(ballInfo != null ? ballInfo.normalizedDistance : 1f);

        sensor.AddObservation(cubeInfo != null && cubeInfo.isVisible ? 1f : 0f);
        sensor.AddObservation(cubeInfo != null ? cubeInfo.horizontalOffset : 0f);
        sensor.AddObservation(cubeInfo != null ? cubeInfo.normalizedDistance : 1f);

        float headNorm = 0f;
        if (trackController != null && trackController.headPlatform != null)
        {
            float rawAngle = trackController.headPlatform.localEulerAngles.y;
            if (rawAngle > 180f) rawAngle -= 360f;
            headNorm = rawAngle / trackController.maxHeadAngle;
        }
        sensor.AddObservation(headNorm);

        sensor.AddObservation(lastKnownTargetDirection); 
        sensor.AddObservation(rb.linearVelocity.magnitude);
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gasSignal   = actions.ContinuousActions[0]; 
        float steerSignal = actions.ContinuousActions[1]; 
        float headSignal  = actions.ContinuousActions[2]; 
        int clawAction    = actions.DiscreteActions[0];

        // 1. Управление виртуальным роботом в симуляции
        if (trackController != null)
            trackController.SetInputs(gasSignal, steerSignal, headSignal);

        // 2. Логика отправки команд в ROS через BehaviorParameters
        if (rosBridge != null && behaviorParameters != null)
        {
            // Проверяем: если стоит HeuristicOnly (WASD) или InferenceOnly (запущенная обученная сетка)
            bool isRealWorldDrive = behaviorParameters.BehaviorType == BehaviorType.HeuristicOnly || 
                                    behaviorParameters.BehaviorType == BehaviorType.InferenceOnly;

            if (isRealWorldDrive)
            {
                // Отправляем газ и руль в ROS
                rosBridge.PublishCmdVel(gasSignal, steerSignal);

                // Отправляем поворот платформы камеры (переводим [-1, 1] в градусов [-90, 90])
                rosBridge.PublishCameraCmd(headSignal * 90f);

                // Отправляем команду клешни (1 - закрыть, 2 - открыть)
                if (clawAction == 1 || clawAction == 2)
                {
                    rosBridge.PublishGripperCmd(clawAction);
                }
            }
        }

        // 3. Вычисление физики, наград и терминальных условий
        ControlClaw(clawAction);
        CalculateRewards(gasSignal, steerSignal);
        CheckTerminalConditions();

        // 4. Проверка на таймаут эпизода
        if (!isEpisodeResolved && MaxStep > 0 && StepCount >= MaxStep - 1)
        {
            SendStatsToTensorBoard(false);
        }
    }

    // НОВОЕ: Единый метод для отправки метрик в TensorBoard
    private void SendStatsToTensorBoard(bool isSuccess)
    {
        if (isEpisodeResolved) return; // Защита от двойной отправки
        
        // Отправляем 1 (100%) если событие случилось, и 0 (0%) если нет. 
        Academy.Instance.StatsRecorder.Add("Custom/1_Grabbed_Ball_Rate", hasGrabbedBall ? 1f : 0f, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Custom/2_Reached_Cube_Rate", isSuccess ? 1f : 0f, StatAggregationMethod.Average);
        
        isEpisodeResolved = true;
    }

    private void CalculateRewards(float gasSignal, float steerSignal)
    {
        bool isHoldingBall = gripperController != null && gripperController.IsHoldingBall();
        
        if (isHoldingBall && !phaseTwoActive)
        {
            phaseTwoActive = true;
            lastKnownTargetDirection = 0f; 
            UpdateTargetMetrics(true); 
        }
        else if (!isHoldingBall && phaseTwoActive)
        {
            phaseTwoActive = false;
            lastKnownTargetDirection = 0f; 
            UpdateTargetMetrics(true); 
        }

        UpdateTargetMetrics(false); 

        // ИЗМЕНЕНО: Агрессивный контраст наград
        if (isHoldingBall)
        {
            AddReward(0.05f); // Огромный плюс каждый кадр за удержание мяча
        }
        else 
        {
            AddReward(-0.005f); // Увеличенный штраф каждый кадр, пока мяч не в руках
        }

        float gasJerk = Mathf.Abs(gasSignal - prevGas);
        float steerJerk = Mathf.Abs(steerSignal - prevSteer);
        AddReward(-(gasJerk + steerJerk) * penaltyJerkMultiplier);
        
        prevGas = gasSignal;
        prevSteer = steerSignal;

        if (virtualSensors != null)
        {
            if (virtualSensors.ultrasoundValue < 0.2f) AddReward(penaltyUltrasound);
            if (virtualSensors.leftIRObstacle == 1 || virtualSensors.rightIRObstacle == 1) AddReward(penaltyIRObstacle);
        }

        // AddReward(rewardTimePenalty);
    }
    
    private void UpdateTargetMetrics(bool forceReset)
    {
        bool isHoldingBall = gripperController != null && gripperController.IsHoldingBall();
        GameObject currentPhysicalTarget = isHoldingBall ? targetCube : targetBall;
        string currentTag = isHoldingBall ? cubeTag : ballTag;

        float distanceReward = 0f;
        float alignmentReward = 0f;

        if (currentPhysicalTarget != null)
        {
            if (gripperController != null && gripperController.holdPoint != null)
            {
                float currentDist = Vector3.Distance(gripperController.holdPoint.position, currentPhysicalTarget.transform.position);
                if (!forceReset)
                {
                    float delta = previousDistanceToTarget - currentDist;
                    distanceReward = delta * rewardDistanceMultiplier;
                    AddReward(distanceReward);
                }
                previousDistanceToTarget = currentDist;
            }

            YoloTargetInfo targetInfo = yoloCamera != null ? yoloCamera.GetTarget(currentTag) : null;
            if (targetInfo != null && targetInfo.isVisible)
            {
                float cameraFactor = 1f - Mathf.Abs(targetInfo.horizontalOffset);
                
                Vector3 dirToTarget = currentPhysicalTarget.transform.position - transform.position;
                dirToTarget.y = 0; 
                float angle = Vector3.Angle(transform.forward, dirToTarget);
                
                float bodyFactor = 0f;
                if (angle <= 20f) bodyFactor = 1f - (angle / 20f); 
                
                float distXZ = dirToTarget.magnitude;
                float distanceFactor = 1f / (1f + distXZ); 
                
                float currentQuality = cameraFactor * bodyFactor * distanceFactor;
                
                if (!forceReset && wasTargetVisibleLastFrame)
                {
                    float deltaQuality = currentQuality - previousTargetQuality;
                    alignmentReward = deltaQuality * rewardCentering;
                    AddReward(alignmentReward);
                }
                
                previousTargetQuality = currentQuality;
                wasTargetVisibleLastFrame = true;
            }
            else
            {
                previousTargetQuality = 0f; 
                wasTargetVisibleLastFrame = false;
            }
        }
    }

    private void CheckTerminalConditions()
    {
        if (targetBall != null && targetCube != null && gripperController != null && !gripperController.IsHoldingBall())
        {
            Vector3 ballPos = targetBall.transform.position;
            ballPos.y = 0; 

            Collider cubeCollider = targetCube.GetComponent<Collider>();
            Vector3 closestPoint = cubeCollider != null ? cubeCollider.ClosestPoint(targetBall.transform.position) : targetCube.transform.position;
            closestPoint.y = 0; 

            float dist = Vector3.Distance(ballPos, closestPoint);

            if (dist <= victoryDistance)
            {
                AddReward(rewardGoalReached);
                SendStatsToTensorBoard(true); // НОВОЕ: Отправка статистики успеха
                Debug.Log($"<color=green>[УСПЕХ]</color> Мяч доставлен на базу! (Дистанция до стенки: {dist:F2})");
                EndEpisode();
            }
        }
    }

    private void ControlClaw(int action)
    {
        if (gripperController == null || virtualSensors == null) return;

        if (action == 1) // Закрыть клешню
        {
            if (virtualSensors.isBallInGripper && targetBall != null)
            {
                gripperController.GrabBall(targetBall);
                
                // Выдаем большую награду ТОЛЬКО если это первый захват в текущем эпизоде
                if (!hasGrabbedBall)
                {
                    AddReward(rewardBallGrabedd); // Единоразовая награда за успешный хват
                    hasGrabbedBall = true; // Фиксируем, чтобы больше не давать награду
                    Debug.Log("<color=cyan>[ФАЗА 1 ЗАВЕРШЕНА]</color> Мяч успешно захвачен!");
                }
            }
            else if (!gripperController.IsHoldingBall())
            {
                if (penalizeEmptyGrab) AddReward(penaltyEmptyGrab);
            }
        }
        else if (action == 2) // Открыть клешню
        {
            if (gripperController.IsHoldingBall())
            {
                gripperController.ReleaseBall();
                
                if (targetCube != null && targetBall != null)
                {
                    Vector3 ballPos = targetBall.transform.position;
                    ballPos.y = 0;
                    
                    Collider cubeCollider = targetCube.GetComponent<Collider>();
                    Vector3 closestPoint = cubeCollider != null ? cubeCollider.ClosestPoint(targetBall.transform.position) : targetCube.transform.position;
                    closestPoint.y = 0;

                    float distToCubeXZ = Vector3.Distance(ballPos, closestPoint);
                    
                    if (distToCubeXZ > victoryDistance)
                    {
                        AddReward(penaltyDropBall); 
                        Debug.Log($"<color=orange>[ШТРАФ]</color> Выронил мяч далеко от базы (Дистанция: {distToCubeXZ:F2} > {victoryDistance}). Можно попробовать дотолкать!");
                    }
                }
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActionsOut = actionsOut.DiscreteActions;
        
        float gas = 0f; float steer = 0f; float headInput = 0f;

        if (Keyboard.current != null)
        {
            var kb = Keyboard.current;
            
            if (kb.wKey.isPressed || kb.upArrowKey.isPressed) gas = 1f;
            else if (kb.sKey.isPressed || kb.downArrowKey.isPressed) gas = -1f;

            if (kb.dKey.isPressed || kb.rightArrowKey.isPressed) steer = 1f;
            else if (kb.aKey.isPressed || kb.leftArrowKey.isPressed) steer = -1f;

            if (kb.eKey.isPressed) headInput = 1f;
            else if (kb.qKey.isPressed) headInput = -1f;

            if (kb.digit1Key.isPressed) discreteActionsOut[0] = 1; 
            else if (kb.digit2Key.isPressed) discreteActionsOut[0] = 2; 
            else discreteActionsOut[0] = 0;
        }

        continuousActionsOut[0] = gas;
        continuousActionsOut[1] = steer;
        continuousActionsOut[2] = headInput;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            AddReward(penaltyCrash); 
            SendStatsToTensorBoard(false); // НОВОЕ: Отправка статистики провала при аварии
            // EndEpisode();     
        }
    }
}