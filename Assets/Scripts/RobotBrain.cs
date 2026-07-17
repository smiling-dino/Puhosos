using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

[RequireComponent(typeof(TrackController))]
public class RobotBrain : Agent
{
    [Header("Mode Selection")]
    [Tooltip("Включите, чтобы перенаправить вводы/выводы на физического робота через ROS.")]
    [SerializeField] private bool useRealRobot = false;

    [Header("Hardware & Controllers (Simulation)")]
    [SerializeField] private SimulatedYoloCamera yoloCamera; //[cite: 9]
    [SerializeField] private Transform cameraServo; //[cite: 9]
    [SerializeField] private VirtualSensors hardwareSensors; //[cite: 9]
    [SerializeField] private GripperController gripperController; //[cite: 9]
    [SerializeField] private ArenaController arenaController; //[cite: 9]

    [Header("Gripper Simulation Only")]
    [SerializeField] private bool hasBall = false; //[cite: 9]
    [SerializeField] private Transform gripperLeftClaw; //[cite: 9]
    [SerializeField] private Transform gripperRightClaw; //[cite: 9]

    [Header("Real Robot Integration")]
    [SerializeField] private RealSensorsReceiver realSensors;
    [SerializeField] private ROSBridge rosBridge;

    private TrackController trackController; //[cite: 9]
    private Rigidbody rb; //[cite: 9]
    
    // Переменные для отслеживания состояния в локальных координатах
    private Vector3 startLocalPosition; //[cite: 9]
    private float previousDistanceToBall = float.MaxValue; //[cite: 9]
    private float lastKnownBallDirection = 0f; //[cite: 9]
    private float timeSinceLastDetection = 0f; //[cite: 9]
    private float lastGasAction = 0f; //[cite: 9]
    private float lastSteerAction = 0f; //[cite: 9]

    public override void Initialize()
    {
        trackController = GetComponent<TrackController>(); //[cite: 9]
        rb = GetComponent<Rigidbody>(); //[cite: 9]
        
        // Запоминаем ЛОКАЛЬНУЮ стартовую позицию относительно нашей арены
        startLocalPosition = transform.localPosition; //[cite: 9]
    }

    public override void OnEpisodeBegin()
    {
        // Сброс базовых переменных состояния (актуально для обоих режимов)
        hasBall = false; //[cite: 9]
        lastKnownBallDirection = 0f; //[cite: 9]
        timeSinceLastDetection = 0f; //[cite: 9]
        lastGasAction = 0f; //[cite: 9]
        lastSteerAction = 0f; //[cite: 9]

        if (useRealRobot)
        {
            // На реальном роботе мы НЕ телепортируем объект и НЕ пересоздаем арену,
            // чтобы не ломать пространственную синхронизацию с физическим миром.
            previousDistanceToBall = GetDistanceToBall();
            return;
        }

        // --- Логика сброса только для Симуляции ---
        trackController.ResetMotors(); //[cite: 9]

        rb.linearVelocity = Vector3.zero; //[cite: 9]
        rb.angularVelocity = Vector3.zero; //[cite: 9]

        if (gripperController != null)
        {
            gripperController.ReleaseBall(); //[cite: 9]
        }

        if (arenaController == null)
        {
            Debug.LogError("ArenaController не назначен", this); //[cite: 9]
            return; //[cite: 9]
        }

        arenaController.ResetArena(); //[cite: 9]

        transform.localRotation = Quaternion.identity; //[cite: 9]
        rb.linearVelocity = Vector3.zero; //[cite: 9]
        rb.angularVelocity = Vector3.zero; //[cite: 9]

        previousDistanceToBall = GetDistanceToBall(); //[cite: 9]
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Получение состояния видимости мяча (YOLO)
        bool isVisible = useRealRobot ? realSensors.isBallVisible : yoloCamera.IsBallVisible(); //[cite: 9]
        float normAngle = useRealRobot ? realSensors.ballNormalizedAngle : yoloCamera.GetNormalizedHorizontalAngle(); //[cite: 9]

        if (isVisible)
        {
            lastKnownBallDirection = Mathf.Sign(normAngle); //[cite: 9]
            timeSinceLastDetection = 0f; //[cite: 9]
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime; //[cite: 9]
        }

        // --- Заполнение вектора наблюдений (строго 15 значений) ---

        // 1. УЗ-дальномер
        float ultrasound = useRealRobot ? realSensors.ultrasoundValue : Mathf.Clamp01(hardwareSensors.ultrasoundValue); //[cite: 9]
        sensor.AddObservation(ultrasound);

        // 2. Левый ИК-датчик препятствий
        float leftIR = useRealRobot ? realSensors.leftIRObstacle : hardwareSensors.leftIRObstacle; //[cite: 9]
        sensor.AddObservation(leftIR);

        // 3. Правый ИК-датчик препятствий
        float rightIR = useRealRobot ? realSensors.rightIRObstacle : hardwareSensors.rightIRObstacle; //[cite: 9]
        sensor.AddObservation(rightIR);

        // 4. ИК-датчик наличия мяча внутри клешни
        float gripperIR = useRealRobot ? (realSensors.isBallInGripper ? 1.0f : 0.0f) : (hardwareSensors.isBallInGripper ? 1.0f : 0.0f); //[cite: 9]
        sensor.AddObservation(gripperIR);

        // 5. Текущий угол до мяча
        sensor.AddObservation(normAngle); //[cite: 9]

        // 6. Дистанция до мяча (нормализованная)
        float distanceToBall = useRealRobot ? realSensors.ballNormalizedDistance : yoloCamera.GetNormalizedDistance(); //[cite: 9]
        sensor.AddObservation(distanceToBall);

        // 7. Направление утерянного мяча
        sensor.AddObservation(lastKnownBallDirection); //[cite: 9]

        // 8. Видимость мяча в кадре (0.0 или 1.0)
        sensor.AddObservation(isVisible ? 1.0f : 0.0f); //[cite: 9]
        
        // 9. Поворот сервопривода камеры
        float servoRotationNorm = cameraServo != null ? cameraServo.localEulerAngles.y / 360f : 0f; //[cite: 9]
        sensor.AddObservation(servoRotationNorm); //[cite: 9]

        // 10. Статус успешного захвата мяча (в реальном режиме синхронизируется с датчиком зажима)
        if (useRealRobot)
        {
            hasBall = realSensors.isBallInGripper;
        }
        sensor.AddObservation(hasBall ? 1.0f : 0.0f); //[cite: 9]
        
        // 11-12. Смещение относительно старта в локальном пространстве арены
        // (Для корректной работы на реальном роботе рекомендуется транслировать одометрию в Transform робота в Unity)
        Vector3 offset = transform.localPosition - startLocalPosition; //[cite: 9]
        sensor.AddObservation(offset.x); // 11. Смещение X[cite: 9]
        sensor.AddObservation(offset.z); // 12. Смещение Z[cite: 9]
        
        // 13. Локальный угол взгляда робота
        float headingAngle = transform.localEulerAngles.y / 360f; //[cite: 9]
        sensor.AddObservation(headingAngle); //[cite: 9]

        // 14. Физическая скорость (в реальном режиме приравниваем к 0, если не симулируем одометрию)
        float speed = useRealRobot ? 0.0f : rb.linearVelocity.magnitude; //[cite: 9]
        sensor.AddObservation(speed); //[cite: 9]

        // 15. Время без визуальных детекций
        sensor.AddObservation(timeSinceLastDetection); //[cite: 9]
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Считывание непрерывных действий
        float gas = actions.ContinuousActions[0]; //[cite: 9]
        float steer = actions.ContinuousActions[1]; //[cite: 9]
        float cameraTurn = actions.ContinuousActions[2]; //[cite: 9]

        if (useRealRobot)
        {
            // Отправляем векторы движения моторов на реального робота через ROS Bridge
            if (rosBridge != null)
            {
                rosBridge.PublishCmdVel(gas, steer);
            }

            // Дублируем движение гусениц в симуляторе для визуального соответствия 3D-модели
            trackController.SetInputs(gas, steer);

            // Поворачиваем виртуальную камеру и шлем угол сервопривода на физического робота
            if (cameraServo != null)
            {
                cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime); //[cite: 9]
                
                // Преобразуем угол вращения Unity в диапазон [-180, 180] градусов для реального сервопривода
                float currentYaw = cameraServo.localEulerAngles.y;
                if (currentYaw > 180f) currentYaw -= 360f;
                
                if (rosBridge != null)
                {
                    rosBridge.PublishCameraCmd(currentYaw);
                }
            }
        }
        else
        {
            // Стандартная автономная симуляция
            trackController.SetInputs(gas, steer); //[cite: 9]

            if (cameraServo != null)
            {
                cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime); //[cite: 9]
            }
        }

        // 2. Считывание дискретного действия (Клешня)
        int gripperAction = actions.DiscreteActions[0]; //[cite: 9]
        ExecuteGripperAction(gripperAction);

        // 3. Расчет наград (выполняется только в симуляторе во время обучения)
        if (!useRealRobot)
        {
            CalculateRewards(gas, steer); //[cite: 9]
        }

        lastGasAction = gas; //[cite: 9]
        lastSteerAction = steer; //[cite: 9]
    }

    private void ExecuteGripperAction(int actionID)
    {
        if (actionID == 1) // Закрыть клешню[cite: 9]
        {
            // Визуальное анимационное отображение сжатия в Unity (для обоих режимов)
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 15, 0); //[cite: 9]
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, -15, 0); //[cite: 9]

            if (useRealRobot)
            {
                if (rosBridge != null)
                {
                    rosBridge.PublishGripperCmd(actionID);
                }
            }
            else
            {
                // Физический захват мяча через GripperController в симуляции
                if (hardwareSensors.isBallInGripper) //[cite: 9]
                {
                    Transform ballTransform = yoloCamera.GetBallTransform(); //[cite: 9]
                    if (ballTransform != null && !gripperController.IsHoldingBall()) //[cite: 9]
                    {
                        gripperController.GrabBall(ballTransform.gameObject); //[cite: 9]
                        hasBall = true; //[cite: 9]
                    }
                }
            }
        }
        else if (actionID == 2) // Открыть клешню[cite: 9]
        {
            // Визуальное разжатие в Unity
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 0, 0); //[cite: 9]
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, 0, 0); //[cite: 9]
            
            if (useRealRobot)
            {
                if (rosBridge != null)
                {
                    rosBridge.PublishGripperCmd(actionID);
                }
                hasBall = false;
            }
            else
            {
                gripperController.ReleaseBall(); //[cite: 9]
                hasBall = false; //[cite: 9]
            }
        }
    }

    private void CalculateRewards(float gas, float steer)
    {
        float currentDistance = GetDistanceToBall(); //[cite: 9]
        float distanceDelta = previousDistanceToBall - currentDistance; //[cite: 9]
        
        if (distanceDelta > 0) //[cite: 9]
        {
            float multiplier = currentDistance < 0.5f ? 2.0f : 1.0f; //[cite: 9]
            AddReward(distanceDelta * multiplier * 0.5f); //[cite: 9]
        }
        previousDistanceToBall = currentDistance; //[cite: 9]

        if (yoloCamera.IsBallVisible()) //[cite: 9]
        {
            float centerDiff = Mathf.Abs(yoloCamera.GetNormalizedHorizontalAngle()); //[cite: 9]
            AddReward((1.0f - centerDiff) * 0.01f); //[cite: 9]
        }

        float gasJerk = Mathf.Abs(gas - lastGasAction); //[cite: 9]
        float steerJerk = Mathf.Abs(steer - lastSteerAction); //[cite: 9]
        AddReward(-(gasJerk + steerJerk) * 0.005f); //[cite: 9]

        // Штраф за опасное сближение со стенами
        if (hardwareSensors.ultrasoundValue < 0.2f || hardwareSensors.leftIRObstacle == 1 || hardwareSensors.rightIRObstacle == 1) //[cite: 9]
        {
            AddReward(-0.02f); //[cite: 9]
        }

        if (hasBall) //[cite: 9]
        {
            SetReward(5.0f); //[cite: 9]
            EndEpisode(); //[cite: 9]
        }

        // Проверка падения или опрокидывания
        float tiltX = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.x)); //[cite: 9]
        float tiltZ = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.z)); //[cite: 9]

        if (transform.localPosition.y < -1f || tiltX > 45f || tiltZ > 45f) //[cite: 9]
        {
            SetReward(-1f); //[cite: 9]
            EndEpisode(); //[cite: 9]
            return; //[cite: 9]
        }
        if (hasBall) //[cite: 9]
        {
            AddReward(5f); //[cite: 9]
            EndEpisode(); //[cite: 9]
            return; //[cite: 9]
        }
    }

    private float GetDistanceToBall()
    {
        if (useRealRobot)
        {
            // На реальном роботе ориентируемся по нормализованному значению с камеры YOLO
            return realSensors.ballNormalizedDistance;
        }

        Transform ball = yoloCamera.GetBallTransform(); //[cite: 9]
        if (ball == null) return 10f; //[cite: 9]
        return Vector3.Distance(transform.localPosition, ball.localPosition); //[cite: 9]
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions; //[cite: 9]
        ActionSegment<int> discrete = actionsOut.DiscreteActions; //[cite: 9]

        continuous[0] = 0f; //[cite: 9]
        continuous[1] = 0f; //[cite: 9]
        continuous[2] = 0f; //[cite: 9]
        discrete[0] = 0; //[cite: 9]

        Keyboard keyboard = Keyboard.current; //[cite: 9]
        if (keyboard == null) //[cite: 9]
        {
            return; //[cite: 9]
        }

        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) //[cite: 9]
            continuous[0] = 1f; //[cite: 9]
        else if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) //[cite: 9]
            continuous[0] = -1f; //[cite: 9]

        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) //[cite: 9]
            continuous[1] = 1f; //[cite: 9]
        else if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) //[cite: 9]
            continuous[1] = -1f; //[cite: 9]

        if (keyboard.eKey.isPressed) //[cite: 9]
            continuous[2] = 1f; //[cite: 9]
        else if (keyboard.qKey.isPressed) //[cite: 9]
            continuous[2] = -1f; //[cite: 9]

        if (keyboard.spaceKey.isPressed) //[cite: 9]
            discrete[0] = 1; //[cite: 9]
        else if (keyboard.leftShiftKey.isPressed) //[cite: 9]
            discrete[0] = 2; //[cite: 9]
    }
    /// <summary>
    /// Метод-заглушка для совместимости со старым скриптом SimpleControl.cs
    /// </summary>
    public void PublishCommand(float gas, float steer)
    {
        rosBridge.PublishCmdVel(gas, steer);
    }
}