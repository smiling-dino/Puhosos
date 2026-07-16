using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class RobotAgent : Agent
{
    [Header("=== Компоненты робота ===")]
    public TrackController trackController;
    public GripperController gripperController;
    public VirtualSensors virtualSensors;
    public SimulatedYoloCamera yoloCamera;
    
    [Header("=== Настройки обучения (Спавн) ===")]
    [Tooltip("Границы случайного спавна по X (от центра арены)")]
    public float spawnAreaWidth = 4f; 
    [Tooltip("Границы случайного спавна по Z (от центра арены)")]
    public float spawnAreaLength = 4f;
    [Tooltip("Высота спавна мяча, чтобы он не провалился под пол")]
    public float ballSpawnHeight = 0.5f;

    [Header("=== Настройки Наград и Штрафов ===")]
    public float rewardGoalReached = 5.0f;        // Успешный захват мяча
    public float rewardCentering = 0.002f;        // Бонус за удержание мяча по центру камеры
    public float rewardDistanceMultiplier = 0.5f; // Множитель награды за сближение с мячом
    
    [Space(10)]
    public float penaltyCrash = -2.0f;            // Штраф за столкновение со стеной
    public float penaltyDropBall = -1.0f;         // Штраф за случайную потерю мяча из клешни
    public float penaltyJerkMultiplier = 0.005f;  // Сила штрафа за резкое управление
    public float penaltyUltrasound = -0.02f;      // Штраф за опасное сближение (УЗ-датчик)
    public float penaltyIRObstacle = -0.01f;      // Штраф за опасное сближение (ИК-датчики)
    
    [Space(10)]
    [Tooltip("Включить штраф за закрытие клешни, если в ней нет мяча?")]
    public bool penalizeEmptyGrab = true;
    public float penaltyEmptyGrab = -0.01f;       // Штраф за пустое смыкание

    private Rigidbody rb;
    private GameObject targetBall;

    // --- Состояния для вычислений ИИ ---
    private Vector3 startPosition;
    private float previousDistanceToBall;
    
    // Память ИИ
    private float prevGas = 0f;
    private float prevSteer = 0f;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.localPosition;
    }

    public override void OnEpisodeBegin()
    {
        // 1. Сброс состояния механизмов
        if (gripperController != null && gripperController.IsHoldingBall())
        {
            gripperController.ReleaseBall();
        }
        
        if (trackController != null)
        {
            trackController.ResetMotors();
        }

        // 2. Сброс переменных и позиции
        prevGas = 0f;
        prevSteer = 0f;
        timeSinceLastDetection = 0f;
        lastKnownBallDirection = 0f;
        
        // Отключаем интерполяцию для телепорта без визуальных глитчей
        rb.interpolation = RigidbodyInterpolation.None; 
        transform.localPosition = startPosition; 
        rb.position = transform.position; 
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        // 3. Поиск мяча на арене (С защитой от пустых ссылок)
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
        else
        {
            Debug.LogError("Ошибка: У агента нет родительского объекта (Арены)!");
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

            previousDistanceToBall = Vector3.Distance(transform.position, targetBall.transform.position);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (yoloCamera != null && yoloCamera.isBallVisible)
        {
            lastKnownBallDirection = yoloCamera.horizontalOffset;
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        sensor.AddObservation(virtualSensors != null ? virtualSensors.ultrasoundValue : 1f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.leftIRObstacle : 0);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.rightIRObstacle : 0);
        sensor.AddObservation(virtualSensors != null && virtualSensors.isBallInGripper ? 1f : 0f);

        sensor.AddObservation(yoloCamera != null ? yoloCamera.horizontalOffset : 0f);
        sensor.AddObservation(yoloCamera != null ? yoloCamera.normalizedDistance : 1f);
        
        sensor.AddObservation(lastKnownBallDirection);
        sensor.AddObservation(yoloCamera != null && yoloCamera.isBallVisible ? 1f : 0f);

        float headNorm = 0f;
        if (trackController != null && trackController.headPlatform != null)
        {
            float rawAngle = trackController.headPlatform.localEulerAngles.y;
            if (rawAngle > 180f) rawAngle -= 360f;
            headNorm = rawAngle / trackController.maxHeadAngle;
        }
        sensor.AddObservation(headNorm);

        sensor.AddObservation(gripperController != null && gripperController.IsHoldingBall() ? 1f : 0f);

        Vector3 offset = transform.localPosition - startPosition;
        sensor.AddObservation(offset.x);
        sensor.AddObservation(offset.z);
        sensor.AddObservation(transform.localEulerAngles.y / 360f);
        sensor.AddObservation(rb.linearVelocity.magnitude);
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gasSignal   = actions.ContinuousActions[0]; 
        float steerSignal = actions.ContinuousActions[1]; 
        float headSignal  = actions.ContinuousActions[2]; 
        int clawAction    = actions.DiscreteActions[0];

        if (trackController != null)
        {
            trackController.SetInputs(gasSignal, steerSignal, headSignal);
        }
        ControlClaw(clawAction);

        CalculateRewards(gasSignal, steerSignal);
        CheckTerminalConditions();
    }

    private void CalculateRewards(float gasSignal, float steerSignal)
    {
        // 1. Награда за сближение
        if (targetBall != null && gripperController != null && !gripperController.IsHoldingBall())
        {
            float currentDist = Vector3.Distance(transform.position, targetBall.transform.position);
            float delta = previousDistanceToBall - currentDist;
            
            if (delta > 0)
            {
                float closenessMult = Mathf.Clamp01(1f - (currentDist / (yoloCamera != null ? yoloCamera.maxVisionDistance : 2f)));
                AddReward(delta * rewardDistanceMultiplier * (1f + closenessMult));
            }
            previousDistanceToBall = currentDist;
        }

        // 2. Штраф за резкость управления
        float gasJerk = Mathf.Abs(gasSignal - prevGas);
        float steerJerk = Mathf.Abs(steerSignal - prevSteer);
        AddReward(-(gasJerk + steerJerk) * penaltyJerkMultiplier);
        
        prevGas = gasSignal;
        prevSteer = steerSignal;

        // 3. Бонус за центрирование
        if (yoloCamera != null && yoloCamera.isBallVisible)
        {
            if (Mathf.Abs(yoloCamera.horizontalOffset) < 0.15f)
            {
                AddReward(rewardCentering);
            }
        }

        // 4. Штраф за датчики расстояния
        if (virtualSensors != null)
        {
            if (virtualSensors.ultrasoundValue < 0.2f) AddReward(penaltyUltrasound);
            if (virtualSensors.leftIRObstacle == 1 || virtualSensors.rightIRObstacle == 1) AddReward(penaltyIRObstacle);
        }
    }

    private void CheckTerminalConditions()
    {
        if (gripperController != null && gripperController.IsHoldingBall())
        {
            AddReward(rewardGoalReached);
            EndEpisode();
        }
    }

    private void ControlClaw(int action)
    {
        if (gripperController == null || virtualSensors == null) return;

        if (action == 1) // Закрыть
        {
            if (virtualSensors.isBallInGripper && targetBall != null)
            {
                gripperController.GrabBall(targetBall);
            }
            else if (!gripperController.IsHoldingBall())
            {
                // Применяем штраф только если галочка включена
                if (penalizeEmptyGrab)
                {
                    AddReward(penaltyEmptyGrab);
                }
            }
        }
        else if (action == 2) // Открыть
        {
            if (gripperController.IsHoldingBall())
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
            EndEpisode();     
        }
    }
}
