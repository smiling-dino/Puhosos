using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("=== Базовые скорости ===")]
    [Tooltip("Максимальная линейная скорость (м/с)")]
    public float moveSpeed = 0.57f;
    [Tooltip("Максимальная скорость поворота (градусов/с)")]
    public float turnSpeed = 120f;

    [Header("=== Дифференциальный привод ===")]
    [Tooltip("Коэффициент влияния руля на гусеницы (0-1)")]
    public float turnK = 0.30f;
    [Tooltip("Лимит поступательной команды (-1 до 1)")]
    public float maxLinearCmd = 0.25f;

    [Header("=== Симуляция моторов (PWM) ===")]
    [Tooltip("Мёртвая зона мотора (0-100%)")]
    public float motorDeadzone = 30f;
    [Tooltip("Минимальный порог старта моторов (PWM)")]
    public float minMotorPwm = 50f;
    [Tooltip("Максимальное изменение PWM за тик (плавность разгона)")]
    public float maxPwmStep = 15f;
    [Tooltip("Коэффициент перевода м/с в PWM")]
    public float pwmMultiplier = 200f;

    [Header("=== Управление головой/башней ===")]
    public Transform headPlatform;
    public float headRotationSpeed = 150f;
    public float maxHeadAngle = 90f;
    private float currentHeadAngle = 0f;

    [Header("=== Коррекция оси движения ===")]
    public Vector3 forwardDirection = Vector3.forward; 

    private Rigidbody rb;
    private float currentLeftPwm = 0f;
    private float currentRightPwm = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.linearDamping = 8f;
        rb.angularDamping = 10f;
        forwardDirection = forwardDirection.normalized;
    }

    // Этот метод теперь принимает ВСЕ команды движения (и для кузова, и для головы)
    public void SetInputs(float gas, float steer, float headSignal)
    {
        // 1. Расчет движения шасси
        gas = Mathf.Clamp(gas, -maxLinearCmd, maxLinearCmd);
        float leftTrackCmd = gas - steer * turnK;
        float rightTrackCmd = gas + steer * turnK;

        float leftPwmTarget = leftTrackCmd * pwmMultiplier;
        float rightPwmTarget = rightTrackCmd * pwmMultiplier;

        leftPwmTarget = ApplyMotorLogic(leftPwmTarget);
        rightPwmTarget = ApplyMotorLogic(rightPwmTarget);

        currentLeftPwm = Mathf.MoveTowards(currentLeftPwm, leftPwmTarget, maxPwmStep);
        currentRightPwm = Mathf.MoveTowards(currentRightPwm, rightPwmTarget, maxPwmStep);

        // 2. Поворот головы
        RotateHead(headSignal);
    }

    private void RotateHead(float rotationSignal)
    {
        if (headPlatform == null) return;

        float step = rotationSignal * headRotationSpeed * Time.fixedDeltaTime;
        currentHeadAngle += step;
        currentHeadAngle = Mathf.Clamp(currentHeadAngle, -maxHeadAngle, maxHeadAngle);
        headPlatform.localRotation = Quaternion.Euler(0f, currentHeadAngle, 0f);
    }

    private float ApplyMotorLogic(float pwm)
    {
        float absPwm = Mathf.Abs(pwm);
        if (absPwm < motorDeadzone) return 0f;
        return Mathf.Sign(pwm) * Mathf.Max(absPwm, minMotorPwm);
    }

    void FixedUpdate()
    {
        float leftSpeed = currentLeftPwm / pwmMultiplier;
        float rightSpeed = currentRightPwm / pwmMultiplier;

        float linearVelocity = (leftSpeed + rightSpeed) / 2f * moveSpeed;
        float angularVelocity = (rightSpeed - leftSpeed) / 2f * turnSpeed;

        Vector3 actualForward = transform.TransformDirection(forwardDirection);

        Vector3 moveVector = actualForward * linearVelocity * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + moveVector);

        Quaternion turnRotation = Quaternion.Euler(0f, angularVelocity * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }

    void OnDrawGizmosSelected()
    {
        if (forwardDirection != Vector3.zero)
        {
            Gizmos.color = Color.green;
            Vector3 actualForward = transform.TransformDirection(forwardDirection);
            Gizmos.DrawRay(transform.position, actualForward * 1f);
        }
    }

    // Добавьте этот метод в любое место внутри класса TrackController
    public void ResetMotors()
    {
        currentLeftPwm = 0f;
        currentRightPwm = 0f;
        
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }
}