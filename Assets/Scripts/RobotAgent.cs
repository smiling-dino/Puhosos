using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class RobotAgent : Agent
{
    [Header("=== Режим работы (Симуляция / Реальный робот) ===")]
    [Tooltip("Включите для управления физическим роботом через ROS.")]
    [SerializeField] private bool useRealRobot = false;
    [SerializeField] private RealSensorsReceiver realSensors; //[cite: 2, 4]
    [SerializeField] private RealVision realVision;             //[cite: 2, 3]
    [SerializeField] private ROSBridge rosBridge;               //[cite: 2, 5]

    [Header("=== Компоненты робота ===")]
    public TrackController trackController;
    public GripperController gripperController;
    public VirtualSensors virtualSensors;
    public SimulatedYoloCamera yoloCamera;
    
    [Header("=== Настройки обучения (Спавн) ===")]
    [Tooltip("Границы случайного спавна по X (от центра арены)")]
    public float spawnAreaWidth = 3.5f;
    [Tooltip("Границы случайного спавна по Z (от центра арены)")]
    public float spawnAreaLength = 3.5f;
    [Tooltip("Высота спавна мяча, чтобы он не провалился под пол")]
    public float ballSpawnHeight = 0.5f;

    [Header("=== Настройки Наград и Штрафов ===")]
    public float rewardGoalReached = 10.0f;        // Успешный захват мяча
    public float rewardCentering = 1.0f;        // Бонус за удержание мяча по центру камеры
    public float rewardDistanceMultiplier = 1.0f; // Множитель награды за сближение с мячом
    
    [Space(10)]
    public float penaltyCrash = -2.0f;            // Штраф за столкновение со стеной
    public float penaltyDropBall = -1.0f;         // Штраф за случайную потерю мяча из клешни
    public float penaltyJerkMultiplier = 0.005f;  // Сила штрафа за резкое управление
    public float penaltyUltrasound = -0.02f;      // Штраф за опасное сближение (УЗ-датчик)
    public float penaltyIRObstacle = -0.01f;      // Штраф за опасное сближение (ИК-датчики)
    public float rewardTimePenalty = -0.002f;       // Штраф за время по модулю
    public float rewardBodyAlignment = 0.002f;    // Бонус за выравнивание корпуса на мяч
    
    [Space(10)]
    [Tooltip("Включить штраф за закрытие клешни, если в ней нет мяча?")]
    public bool penalizeEmptyGrab = true;
    public float penaltyEmptyGrab = -0.01f;       // Штраф за пустое смыкание

    private Rigidbody rb;
    private GameObject targetBall;

    // --- Состояния для вычислений ИИ ---
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float previousDistanceToBall;
    
    // Память ИИ
    private float prevGas = 0f;
    private float prevSteer = 0f;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private float previousCameraOffset = 1f;
    private bool isEpisodeResolved = false;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.localPosition;
        startRotation = transform.localRotation;
    }

    public override void OnEpisodeBegin()
    {
        isEpisodeResolved = false;

        // Если это реальный робот, мы не телепортируем объект, чтобы не ломать синхронизацию
        if (useRealRobot)
        {
            previousDistanceToBall = GetCurrentDistanceToBall();
            return;
        }

        // 1. Сброс состояния механизмов через встроенный в TrackController метод
        if (gripperController != null && gripperController.IsHoldingBall())
        {
            gripperController.ReleaseBall();
        }
        
        if (trackController != null)
        {
            trackController.ResetMotors(); //[cite: 7]
        }

        // 2. Сброс переменных и позиции
        prevGas = 0f;
        prevSteer = 0f;
        timeSinceLastDetection = 0f;
        lastKnownBallDirection = 0f;
        
        rb.interpolation = RigidbodyInterpolation.None; 

        transform.localPosition = startPosition; 
        transform.localRotation = startRotation; 

        rb.position = transform.position; 
        rb.rotation = transform.rotation;        

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        // 3. Поиск мяча на арене
        targetBall = null; 
        Transform arena = transform.parent; 
        if (arena != null) 
        {
            if (virtualSensors == null)
            {
                Debug.LogError($"[RobotAgent] У робота {gameObject.name} не назначен VirtualSensors! Поиск мяча невозможен.");
                return;
            }

            foreach (Transform child in arena)
            {
                if (child.CompareTag(virtualSensors.ballTag))
                {
                    targetBall = child.gameObject;
                    break;
                }
            }
        }

        // 4. Спавн мяча в новой позиции
        if (targetBall != null)
        {
            float randomX = Random.Range(-spawnAreaWidth, spawnAreaWidth);
            float randomZ = Random.Range(-spawnAreaLength, spawnAreaLength);
            targetBall.transform.localPosition = new Vector3(randomX, ballSpawnHeight, randomZ);

            Rigidbody ballRb = targetBall.GetComponent<Rigidbody>();
            if (ballRb != null)
            {
                ballRb.linearVelocity = Vector3.zero;
                ballRb.angularVelocity = Vector3.zero;
            }

            previousDistanceToBall = GetCurrentDistanceToBall();
            previousCameraOffset = 1f; 
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Источники данных зависят от режима (Симуляция vs Реальный робот)[cite: 2]
        bool isVisible = useRealRobot ? realVision.seesBall : (yoloCamera != null && yoloCamera.isBallVisible);
        float normAngle = useRealRobot ? realVision.normalizedAngle : (yoloCamera != null ? yoloCamera.horizontalOffset : 0f);
        float normDist = useRealRobot ? realVision.normalizedDistance : (yoloCamera != null ? yoloCamera.normalizedDistance : 1f);

        if (isVisible)
        {
            lastKnownBallDirection = normAngle;
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        // Сбор показаний датчиков
        float ultrasound = useRealRobot ? realSensors.ultrasoundValue : (virtualSensors != null ? virtualSensors.ultrasoundValue : 1f);
        float leftIR = useRealRobot ? realSensors.leftIRObstacle : (virtualSensors != null ? virtualSensors.leftIRObstacle : 0);
        float rightIR = useRealRobot ? realSensors.rightIRObstacle : (virtualSensors != null ? virtualSensors.rightIRObstacle : 0);
        bool ballInGripper = useRealRobot ? realSensors.isBallInGripper : (virtualSensors != null ? virtualSensors.isBallInGripper : false);

        sensor.AddObservation(ultrasound);
        sensor.AddObservation(leftIR);
        sensor.AddObservation(rightIR);
        sensor.AddObservation(ballInGripper ? 1f : 0f);

        sensor.AddObservation(normAngle);
        sensor.AddObservation(normDist);
        
        sensor.AddObservation(lastKnownBallDirection);
        sensor.AddObservation(isVisible ? 1f : 0f);

        // Нормализация угла башни (головы) на основе лимитов из TrackController
        float headNorm = 0f;
        if (trackController != null && trackController.headPlatform != null)
        {
            float rawAngle = trackController.headPlatform.localEulerAngles.y;
            if (rawAngle > 180f) rawAngle -= 360f;
            headNorm = rawAngle / trackController.maxHeadAngle; //[cite: 7]
        }
        sensor.AddObservation(headNorm);

        bool isHolding = useRealRobot ? realSensors.isBallInGripper : (gripperController != null && gripperController.IsHoldingBall());
        sensor.AddObservation(isHolding ? 1f : 0f);

        Vector3 offset = transform.localPosition - startPosition;
        sensor.AddObservation(offset.x);
        sensor.AddObservation(offset.z);
        sensor.AddObservation(transform.localEulerAngles.y / 360f);
        sensor.AddObservation(useRealRobot ? 0f : rb.linearVelocity.magnitude);
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gasSignal   = actions.ContinuousActions[0]; 
        float steerSignal = actions.ContinuousActions[1]; 
        float headSignal  = actions.ContinuousActions[2]; 
        int clawAction    = actions.DiscreteActions[0];

        if (useRealRobot)
        {
            // Отправляем команды на физического робота через ROS Bridge[cite: 2, 5]
            if (rosBridge != null)
            {
                rosBridge.PublishCmdVel(gasSignal, steerSignal);     //[cite: 5]
                rosBridge.PublishGripperCmd(clawAction);              //[cite: 5]
            }

            // Дублируем движение в симуляторе через TrackController для визуализации
            if (trackController != null)
            {
                trackController.SetInputs(gasSignal, steerSignal, headSignal); //[cite: 7]
                
                // Передаем текущий угол поворота головы в ROS
                if (rosBridge != null && trackController.headPlatform != null)
                {
                    float currentYaw = trackController.headPlatform.localEulerAngles.y;
                    if (currentYaw > 180f) currentYaw -= 360f;
                    rosBridge.PublishCameraCmd(currentYaw); //[cite: 5]
                }
            }
        }
        else
        {
            // Обычный режим симуляции
            if (trackController != null)
            {
                trackController.SetInputs(gasSignal, steerSignal, headSignal); //[cite: 7]
            }
            ControlClaw(clawAction);

            CalculateRewards(gasSignal, steerSignal);
            CheckTerminalConditions();

            // Проверка на тайм-аут
            if (!isEpisodeResolved && MaxStep > 0 && StepCount >= MaxStep - 1)
            {
                Academy.Instance.StatsRecorder.Add("Custom/Success Rate", 0f, StatAggregationMethod.Average);
                isEpisodeResolved = true;
            }
        }
    }

    private float GetCurrentDistanceToBall()
    {
        if (useRealRobot)
        {
            return realVision.normalizedDistance; //[cite: 2, 3]
        }

        if (targetBall != null && gripperController != null && gripperController.holdPoint != null)
        {
            return Vector3.Distance(gripperController.holdPoint.position, targetBall.transform.position);
        }
        return 10f;
    }

    private void CalculateRewards(float gasSignal, float steerSignal)
    {
        // 1. Дельта-награда за сближение клешни с мячом
        float currentDist = GetCurrentDistanceToBall();
        float delta = previousDistanceToBall - currentDist;
        AddReward(delta * rewardDistanceMultiplier);
        previousDistanceToBall = currentDist;

        // 2. Дельта-награда за центрирование камеры (YOLO)
        if (yoloCamera != null && yoloCamera.isBallVisible)
        {
            float currentOffset = Mathf.Abs(yoloCamera.horizontalOffset);
            float deltaOffset = previousCameraOffset - currentOffset;
            AddReward(deltaOffset * rewardCentering);
            previousCameraOffset = currentOffset;
        }
        else
        {
            previousCameraOffset = 1f; 
        }

        // 3. Штраф за резкость управления
        float gasJerk = Mathf.Abs(gasSignal - prevGas);
        float steerJerk = Mathf.Abs(steerSignal - prevSteer);
        AddReward(-(gasJerk + steerJerk) * penaltyJerkMultiplier);
        
        prevGas = gasSignal;
        prevSteer = steerSignal;

        // 4. Штрафы с датчиков (столкновения)
        if (virtualSensors != null)
        {
            if (virtualSensors.ultrasoundValue < 0.2f) AddReward(penaltyUltrasound);
            if (virtualSensors.leftIRObstacle == 1 || virtualSensors.rightIRObstacle == 1) AddReward(penaltyIRObstacle);
        }

        // 5. Штраф за время
        AddReward(rewardTimePenalty);
    }

    private void CheckTerminalConditions()
    {
        if (gripperController != null && gripperController.IsHoldingBall())
        {
            AddReward(rewardGoalReached);
            
            if (!isEpisodeResolved)
            {
                Academy.Instance.StatsRecorder.Add("Custom/Success Rate", 1f, StatAggregationMethod.Average);
                isEpisodeResolved = true;
            }
            
            EndEpisode();
        }
    }

    private void ControlClaw(int action)
    {
        if (virtualSensors == null) return;

        if (action == 1) // Закрыть
        {
            if (virtualSensors.isBallInGripper && targetBall != null)
            {
                gripperController.GrabBall(targetBall);
            }
            else if (gripperController != null && !gripperController.IsHoldingBall())
            {
                if (penalizeEmptyGrab) AddReward(penaltyEmptyGrab);
            }
        }
        else if (action == 2) // Открыть
        {
            if (gripperController != null && gripperController.IsHoldingBall())
            {
                gripperController.ReleaseBall();
                AddReward(penaltyDropBall); 
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

            if (kb.dKey.isPressed || kb.rightArrowKey.isPressed) steer = -1f;
            else if (kb.aKey.isPressed || kb.leftArrowKey.isPressed) steer = 1f;

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
        if (!useRealRobot && collision.gameObject.CompareTag("Obstacle"))
        {
            AddReward(penaltyCrash); 
            
            if (!isEpisodeResolved)
            {
                Academy.Instance.StatsRecorder.Add("Custom/Success Rate", 0f, StatAggregationMethod.Average);
                isEpisodeResolved = true;
            }
            
            EndEpisode();     
        }
    }
}