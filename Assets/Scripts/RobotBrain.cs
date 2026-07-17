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
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraServo;
    [SerializeField] private VirtualSensors hardwareSensors;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private ArenaController arenaController;

    [Header("Gripper Simulation Only")]
    [SerializeField] private bool hasBall = false;
    [SerializeField] private Transform gripperLeftClaw;
    [SerializeField] private Transform gripperRightClaw;

    [Header("Real Robot Integration")]
    [SerializeField] private RealSensorsReceiver realSensors;
    [SerializeField] private RealVision realVision; // НАШЕ НОВОЕ ПОЛЕ ДЛЯ ПОДКЛЮЧЕНИЯ YOLO С ПК[cite: 1, 2]
    [SerializeField] private ROSBridge rosBridge;

    private TrackController trackController;
    private Rigidbody rb;
    
    // Переменные для отслеживания состояния в локальных координатах
    private Vector3 startLocalPosition;
    private float previousDistanceToBall = float.MaxValue;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private float lastGasAction = 0f;
    private float lastSteerAction = 0f;

    public override void Initialize()
    {
        trackController = GetComponent<TrackController>();
        rb = GetComponent<Rigidbody>();
        
        // Запоминаем ЛОКАЛЬНУЮ стартовую позицию относительно нашей арены
        startLocalPosition = transform.localPosition;
    }

    public override void OnEpisodeBegin()
    {
        // Сброс базовых переменных состояния (актуально для обоих режимов)
        hasBall = false;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        lastGasAction = 0f;
        lastSteerAction = 0f;

        if (useRealRobot)
        {
            // На реальном роботе мы НЕ телепортируем объект и НЕ пересоздаем арену,
            // чтобы не ломать пространственную синхронизацию с физическим миром.
            previousDistanceToBall = GetDistanceToBall();
            return;
        }

        // --- Логика сброса только для Симуляции ---
        trackController.ResetMotors();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (gripperController != null)
        {
            gripperController.ReleaseBall();
        }

        if (arenaController == null)
        {
            Debug.LogError("ArenaController не назначен", this);
            return;
        }

        arenaController.ResetArena();

        transform.localRotation = Quaternion.identity;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        previousDistanceToBall = GetDistanceToBall();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Получение состояния видимости мяча (Переключаемся на RealVision для реального робота)[cite: 1, 2]
        bool isVisible = useRealRobot ? realVision.seesBall : yoloCamera.IsBallVisible();//[cite: 1, 2]
        float normAngle = useRealRobot ? realVision.normalizedAngle : yoloCamera.GetNormalizedHorizontalAngle();//[cite: 1, 2]

        if (isVisible)
        {
            lastKnownBallDirection = Mathf.Sign(normAngle);
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        // --- Заполнение вектора наблюдений (строго 15 значений) ---

        // 1. УЗ-дальномер
        float ultrasound = useRealRobot ? realSensors.ultrasoundValue : Mathf.Clamp01(hardwareSensors.ultrasoundValue);
        sensor.AddObservation(ultrasound);

        // 2. Левый ИК-датчик препятствий
        float leftIR = useRealRobot ? realSensors.leftIRObstacle : hardwareSensors.leftIRObstacle;
        sensor.AddObservation(leftIR);

        // 3. Правый ИК-датчик препятствий
        float rightIR = useRealRobot ? realSensors.rightIRObstacle : hardwareSensors.rightIRObstacle;
        sensor.AddObservation(rightIR);

        // 4. ИК-датчик наличия мяча внутри клешни
        float gripperIR = useRealRobot ? (realSensors.isBallInGripper ? 1.0f : 0.0f) : (hardwareSensors.isBallInGripper ? 1.0f : 0.0f);
        sensor.AddObservation(gripperIR);

        // 5. Текущий угол до мяча
        sensor.AddObservation(normAngle);

        // 6. Дистанция до мяча (Берем из RealVision, транслирующего данные сети)[cite: 1, 2]
        float distanceToBall = useRealRobot ? realVision.normalizedDistance : yoloCamera.GetNormalizedDistance();//[cite: 1, 2]
        sensor.AddObservation(distanceToBall);

        // 7. Направление утерянного мяча
        sensor.AddObservation(lastKnownBallDirection);

        // 8. Видимость мяча в кадре (0.0 или 1.0)
        sensor.AddObservation(isVisible ? 1.0f : 0.0f);
        
        // 9. Поворот сервопривода камеры
        float servoRotationNorm = cameraServo != null ? cameraServo.localEulerAngles.y / 360f : 0f;
        sensor.AddObservation(servoRotationNorm);

        // 10. Статус успешного захвата мяча (в реальном режиме синхронизируется с датчиком зажима)
        if (useRealRobot)
        {
            hasBall = realSensors.isBallInGripper;
        }
        sensor.AddObservation(hasBall ? 1.0f : 0.0f);
        
        // 11-12. Смещение относительно старта в локальном пространстве арены
        Vector3 offset = transform.localPosition - startLocalPosition;
        sensor.AddObservation(offset.x); // 11. Смещение X
        sensor.AddObservation(offset.z); // 12. Смещение Z
        
        // 13. Локальный угол взгляда робота
        float headingAngle = transform.localEulerAngles.y / 360f;
        sensor.AddObservation(headingAngle);

        // 14. Физическая скорость (в реальном режиме приравниваем к 0, если не симулируем одометрию)
        float speed = useRealRobot ? 0.0f : rb.linearVelocity.magnitude;
        sensor.AddObservation(speed);

        // 15. Время без визуальных детекций
        sensor.AddObservation(timeSinceLastDetection);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Считывание непрерывных действий
        float gas = actions.ContinuousActions[0];
        float steer = actions.ContinuousActions[1];
        float cameraTurn = actions.ContinuousActions[2];

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
                cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime);
                
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
            trackController.SetInputs(gas, steer);

            if (cameraServo != null)
            {
                cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime);
            }
        }

        // 2. Считывание дискретного действия (Клешня)
        int gripperAction = actions.DiscreteActions[0];
        ExecuteGripperAction(gripperAction);

        // 3. Расчет наград (выполняется только в симуляторе во время обучения)
        if (!useRealRobot)
        {
            CalculateRewards(gas, steer);
        }

        lastGasAction = gas;
        lastSteerAction = steer;
    }

    private void ExecuteGripperAction(int actionID)
    {
        if (actionID == 1) // Закрыть клешню
        {
            // Визуальное анимационное отображение сжатия в Unity (для обоих режимов)
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 15, 0);
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, -15, 0);

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
                if (hardwareSensors.isBallInGripper)
                {
                    Transform ballTransform = yoloCamera.GetBallTransform();
                    if (ballTransform != null && !gripperController.IsHoldingBall())
                    {
                        gripperController.GrabBall(ballTransform.gameObject);
                        hasBall = true;
                    }
                }
            }
        }
        else if (actionID == 2) // Открыть клешню
        {
            // Визуальное разжатие в Unity
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 0, 0);
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, 0, 0);
            
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
                gripperController.ReleaseBall();
                hasBall = false;
            }
        }
    }

    private void CalculateRewards(float gas, float steer)
    {
        float currentDistance = GetDistanceToBall();
        float distanceDelta = previousDistanceToBall - currentDistance;
        
        if (distanceDelta > 0)
        {
            float multiplier = currentDistance < 0.5f ? 2.0f : 1.0f;
            AddReward(distanceDelta * multiplier * 0.5f);
        }
        previousDistanceToBall = currentDistance;

        if (yoloCamera.IsBallVisible())
        {
            float centerDiff = Mathf.Abs(yoloCamera.GetNormalizedHorizontalAngle());
            AddReward((1.0f - centerDiff) * 0.01f);
        }

        float gasJerk = Mathf.Abs(gas - lastGasAction);
        float steerJerk = Mathf.Abs(steer - lastSteerAction);
        AddReward(-(gasJerk + steerJerk) * 0.005f);

        // Штраф за опасное сближение со стенами
        if (hardwareSensors.ultrasoundValue < 0.2f || hardwareSensors.leftIRObstacle == 1 || hardwareSensors.rightIRObstacle == 1)
        {
            AddReward(-0.02f);
        }

        if (hasBall)
        {
            SetReward(5.0f);
            EndEpisode();
        }

        // Проверка падения или опрокидывания
        float tiltX = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.x));
        float tiltZ = Mathf.Abs(Mathf.DeltaAngle(0f, transform.localEulerAngles.z));

        if (transform.localPosition.y < -1f || tiltX > 45f || tiltZ > 45f)
        {
            SetReward(-1f);
            EndEpisode();
            return;
        }
        if (hasBall)
        {
            AddReward(5f);
            EndEpisode();
            return;
        }
    }

    private float GetDistanceToBall()
    {
        if (useRealRobot)
        {
            // Ориентируемся по нормализованному значению с физической RealVision[cite: 1, 2]
            return realVision.normalizedDistance;//[cite: 1, 2]
        }

        Transform ball = yoloCamera.GetBallTransform();
        if (ball == null) return 10f;
        return Vector3.Distance(transform.localPosition, ball.localPosition);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions;
        ActionSegment<int> discrete = actionsOut.DiscreteActions;

        continuous[0] = 0f;
        continuous[1] = 0f;
        continuous[2] = 0f;
        discrete[0] = 0;

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

    /// <summary>
    /// Метод-заглушка для совместимости со старым скриптом SimpleControl.cs
    /// </summary>
    public void PublishCommand(float gas, float steer)
    {
        if (rosBridge != null)
        {
            rosBridge.PublishCmdVel(gas, steer);
        }
    }
}