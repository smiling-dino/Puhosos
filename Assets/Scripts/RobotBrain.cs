using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

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
        trackController.ResetMotors();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (gripperController != null)
        {
            gripperController.ReleaseBall();
        }

        hasBall = false;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        lastGasAction = 0f;
        lastSteerAction = 0f;

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
        bool isVisible = yoloCamera.IsBallVisible();
        float normAngle = yoloCamera.GetNormalizedHorizontalAngle();

        if (isVisible)
        {
            lastKnownBallDirection = Mathf.Sign(normAngle);
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.fixedDeltaTime;
        }

        // Заполнение вектора наблюдений (строго 15 значений)
        sensor.AddObservation(Mathf.Clamp01(hardwareSensors.ultrasoundValue));            // 1. УЗ-дальномер
        sensor.AddObservation(hardwareSensors.leftIRObstacle);                            // 2. Левый ИК
        sensor.AddObservation(hardwareSensors.rightIRObstacle);                           // 3. Правый ИК
        sensor.AddObservation(hardwareSensors.isBallInGripper ? 1.0f : 0.0f);             // 4. ИК в клешне
        sensor.AddObservation(normAngle);                                                 // 5. Угол до мяча
        sensor.AddObservation(yoloCamera.GetNormalizedDistance());                        // 6. Дистанция до мяча
        sensor.AddObservation(lastKnownBallDirection);                                    // 7. Направление утерянного мяча
        sensor.AddObservation(isVisible ? 1.0f : 0.0f);                                   // 8. Видимость мяча
        
        float servoRotationNorm = cameraServo != null ? cameraServo.localEulerAngles.y / 360f : 0f;
        sensor.AddObservation(servoRotationNorm);                                         // 9. Поворот серво камеры

        sensor.AddObservation(hasBall ? 1.0f : 0.0f);                                     // 10. Статус захвата мяча
        
        // Смещение считаем в локальном пространстве арены
        Vector3 offset = transform.localPosition - startLocalPosition;
        sensor.AddObservation(offset.x);                                                  // 11. Смещение X
        sensor.AddObservation(offset.z);                                                  // 12. Смещение Z
        
        float headingAngle = transform.localEulerAngles.y / 360f;
        sensor.AddObservation(headingAngle);                                              // 13. Угол взгляда робота (локальный)
        sensor.AddObservation(rb.linearVelocity.magnitude);                               // 14. Физическая скорость
        sensor.AddObservation(timeSinceLastDetection);                                    // 15. Время без детекций
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Считывание непрерывных действий
        float gas = actions.ContinuousActions[0];
        float steer = actions.ContinuousActions[1];
        float cameraTurn = actions.ContinuousActions[2];

        // Применяем движение к моторам гусениц
        trackController.SetInputs(gas, steer);

        // Поворот камеры
        if (cameraServo != null)
        {
            cameraServo.Rotate(Vector3.up, cameraTurn * 50f * Time.fixedDeltaTime);
        }

        // 2. Считывание дискретного действия (Клешня)
        int gripperAction = actions.DiscreteActions[0];
        ExecuteGripperAction(gripperAction);

        // 3. Расчет наград
        CalculateRewards(gas, steer);

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
        else if (actionID == 2) // Открыть
        {
            if (gripperLeftClaw != null) gripperLeftClaw.localRotation = Quaternion.Euler(0, 0, 0);
            if (gripperRightClaw != null) gripperRightClaw.localRotation = Quaternion.Euler(0, 0, 0);
            
            gripperController.ReleaseBall();
            hasBall = false;
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
        float tiltX = Mathf.Abs(
            Mathf.DeltaAngle(0f, transform.localEulerAngles.x)
        );

        float tiltZ = Mathf.Abs(
            Mathf.DeltaAngle(0f, transform.localEulerAngles.z)
        );

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
}