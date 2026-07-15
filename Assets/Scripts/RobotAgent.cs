using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class RobotAgent : Agent
{
    [Header("=== Компоненты робота ===")]
    public TrackController trackController; //[cite: 13]
    public GripperController gripperController; //[cite: 16]
    public VirtualSensors virtualSensors; //[cite: 15]
    public SimulatedYoloCamera yoloCamera; //[cite: 12]

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
        if (gripperController != null && gripperController.IsHoldingBall()) //[cite: 16]
        {
            gripperController.ReleaseBall(); //[cite: 16]
        }
        
        if (trackController != null)
        {
            trackController.ResetMotors(); //[cite: 13]
        }

        // 2. Сброс переменных и позиции
        prevGas = 0f;
        prevSteer = 0f;
        timeSinceLastDetection = 0f;
        lastKnownBallDirection = 0f;
        
        transform.localPosition = startPosition; 
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // 3. Поиск мяча на арене
        Transform arena = transform.parent; 
        if (arena != null) 
        {
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

        if (targetBall != null)
        {
            previousDistanceToBall = Vector3.Distance(transform.position, targetBall.transform.position);
        }
    }

    /// <summary>
    /// ШАГ 1: СБОР НАБЛЮДЕНИЙ (Строго 15 параметров)
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // Обновление памяти о мяче на основе данных камеры
        if (yoloCamera != null && yoloCamera.isBallVisible) //[cite: 12]
        {
            lastKnownBallDirection = yoloCamera.horizontalOffset; //[cite: 12]
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        // 1. Нормализованное расстояние с УЗ-дальномера
        sensor.AddObservation(virtualSensors != null ? virtualSensors.ultrasoundValue : 1f); //[cite: 15]
        
        // 2-3. Левый и Правый ИК-датчики препятствий (0/1)
        sensor.AddObservation(virtualSensors != null ? virtualSensors.leftIRObstacle : 0); //[cite: 15]
        sensor.AddObservation(virtualSensors != null ? virtualSensors.rightIRObstacle : 0); //[cite: 15]
        
        // 4. ИК-датчик клешни (0/1)
        sensor.AddObservation(virtualSensors != null && virtualSensors.isBallInGripper ? 1f : 0f); //[cite: 15]

        // 5-6. Зрение YOLO: угол и расстояние 
        sensor.AddObservation(yoloCamera != null ? yoloCamera.horizontalOffset : 0f); //[cite: 12]
        sensor.AddObservation(yoloCamera != null ? yoloCamera.normalizedDistance : 1f); //[cite: 12]
        
        // 7-8. Память: Последнее направление и видимость
        sensor.AddObservation(lastKnownBallDirection);
        sensor.AddObservation(yoloCamera != null && yoloCamera.isBallVisible ? 1f : 0f); //[cite: 12]

        // 9. Текущий поворот сервопривода (Нормализованный от -1 до 1)
        float headNorm = 0f;
        if (trackController != null && trackController.headPlatform != null) //[cite: 13]
        {
            float rawAngle = trackController.headPlatform.localEulerAngles.y; //[cite: 13]
            if (rawAngle > 180f) rawAngle -= 360f;
            headNorm = rawAngle / trackController.maxHeadAngle; //[cite: 13]
        }
        sensor.AddObservation(headNorm);

        // 10. Статус захвата мяча клешнёй (0.0 или 1.0)
        sensor.AddObservation(gripperController != null && gripperController.IsHoldingBall() ? 1f : 0f); //[cite: 16]

        // 11-12. Относительное смещение робота по оси X и Z
        Vector3 offset = transform.localPosition - startPosition;
        sensor.AddObservation(offset.x);
        sensor.AddObservation(offset.z);

        // 13. Нормализованный угол направления взгляда робота (Heading)
        sensor.AddObservation(transform.eulerAngles.y / 360f);

        // 14. Текущая скорость движения
        sensor.AddObservation(rb.linearVelocity.magnitude);

        // 15. Время, прошедшее с момента последней детекции мяча (нормализованное, макс 10 сек)
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    /// <summary>
    /// ШАГ 2: ПРИЕМ ДЕЙСТВИЙ И УПРАВЛЕНИЕ
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Считываем сигналы
        float gasSignal   = actions.ContinuousActions[0]; 
        float steerSignal = actions.ContinuousActions[1]; 
        float headSignal  = actions.ContinuousActions[2]; 
        int clawAction    = actions.DiscreteActions[0];

        // Отправляем команды в контроллеры
        if (trackController != null)
        {
            trackController.SetInputs(gasSignal, steerSignal, headSignal); //[cite: 13]
        }
        ControlClaw(clawAction);

        // ШАГ 4: Расчет наград (Rewards)
        CalculateRewards(gasSignal, steerSignal);
        CheckTerminalConditions();
    }

    private void CalculateRewards(float gasSignal, float steerSignal)
    {
        // 1. Награда за сближение (Distance Delta)
        if (targetBall != null && gripperController != null && !gripperController.IsHoldingBall()) //[cite: 16]
        {
            float currentDist = Vector3.Distance(transform.position, targetBall.transform.position);
            float delta = previousDistanceToBall - currentDist;
            
            if (delta > 0)
            {
                // Чем ближе мяч, тем выше множитель (до x2)
                float closenessMult = Mathf.Clamp01(1f - (currentDist / (yoloCamera != null ? yoloCamera.maxVisionDistance : 2f))); //[cite: 12]
                AddReward(delta * 0.5f * (1f + closenessMult));
            }
            previousDistanceToBall = currentDist;
        }

        // 2. Штраф за резкость управления (Action Rate Penalty)
        float gasJerk = Mathf.Abs(gasSignal - prevGas);
        float steerJerk = Mathf.Abs(steerSignal - prevSteer);
        AddReward(-(gasJerk + steerJerk) * 0.005f); // Плавное управление = меньше штраф
        
        prevGas = gasSignal;
        prevSteer = steerSignal;

        // 3. Бонус за центрирование
        if (yoloCamera != null && yoloCamera.isBallVisible) //[cite: 12]
        {
            // Если мяч почти по центру камеры
            if (Mathf.Abs(yoloCamera.horizontalOffset) < 0.15f) //[cite: 12]
            {
                AddReward(0.002f);
            }
        }

        // 4. Штраф за датчики расстояния (чтобы не терся об стены)
        if (virtualSensors != null)
        {
            if (virtualSensors.ultrasoundValue < 0.2f) AddReward(-0.02f); //[cite: 15]
            if (virtualSensors.leftIRObstacle == 1 || virtualSensors.rightIRObstacle == 1) AddReward(-0.01f); //[cite: 15]
        }
    }

    private void CheckTerminalConditions()
    {
        // Успешный финал
        if (gripperController != null && gripperController.IsHoldingBall()) //[cite: 1]
        {
            AddReward(5.0f); //[cite: 1]
            EndEpisode(); //[cite: 1]
        }

        // Проверку на падение (localPosition.y < -1f) мы отсюда удалили
    }

    private void ControlClaw(int action)
    {
        if (gripperController == null || virtualSensors == null) return;

        if (action == 1) // Закрыть
        {
            if (virtualSensors.isBallInGripper && targetBall != null) //[cite: 15]
            {
                gripperController.GrabBall(targetBall); //[cite: 16]
            }
            else if (!gripperController.IsHoldingBall()) //[cite: 16]
            {
                AddReward(-0.01f); // Легкий штраф за щелканье впустую
            }
        }
        else if (action == 2) // Открыть
        {
            if (gripperController.IsHoldingBall()) //[cite: 16]
            {
                gripperController.ReleaseBall(); //[cite: 16]
                AddReward(-1.0f); // Штраф за потерю мяча до конца эпизода
            }
        }
    }

    /// <summary>
    /// РУЧНОЕ УПРАВЛЕНИЕ (Для тестирования)
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions; //[cite: 14]
        var discreteActionsOut = actionsOut.DiscreteActions; //[cite: 14]
        
        float gas = 0f; float steer = 0f; float headInput = 0f;

        if (Keyboard.current != null) //[cite: 14]
        {
            var kb = Keyboard.current; //[cite: 14]
            
            // Движение
            if (kb.wKey.isPressed || kb.upArrowKey.isPressed) gas = 1f; //[cite: 14]
            else if (kb.sKey.isPressed || kb.downArrowKey.isPressed) gas = -1f; //[cite: 14]

            // Поворот
            if (kb.dKey.isPressed || kb.rightArrowKey.isPressed) steer = 1f; //[cite: 14]
            else if (kb.aKey.isPressed || kb.leftArrowKey.isPressed) steer = -1f; //[cite: 14]

            // Камера
            if (kb.eKey.isPressed) headInput = 1f; //[cite: 14]
            else if (kb.qKey.isPressed) headInput = -1f; //[cite: 14]

            // Клешня
            if (kb.digit1Key.isPressed) discreteActionsOut[0] = 1;  //[cite: 14]
            else if (kb.digit2Key.isPressed) discreteActionsOut[0] = 2;  //[cite: 14]
            else discreteActionsOut[0] = 0;  //[cite: 14]
        }

        continuousActionsOut[0] = gas; //[cite: 14]
        continuousActionsOut[1] = steer; //[cite: 14]
        continuousActionsOut[2] = headInput; //[cite: 14]
    }

    /// <summary>
    /// Отслеживание физических столкновений
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        // Если мы коснулись целевого мяча или пола, игнорируем это.
        // Нас интересуют только стены и препятствия.
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            AddReward(-2.0f); // Даем сильный штраф за аварию
            EndEpisode();     // Сбрасываем сцену
        }
    }
}