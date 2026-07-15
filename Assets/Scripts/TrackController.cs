using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("=== Базовые скорости ===")]
    public float moveSpeed = 0.57f;
    public float turnSpeed = 120f;

    [Header("=== Дифференциальный привод ===")]
    public float turnK = 0.30f;
    public float maxLinearCmd = 0.25f;

    [Header("=== Симуляция моторов (PWM) ===")]
    public float motorDeadzone = 30f;
    public float minMotorPwm = 50f;
    public float maxPwmStep = 15f;
    public float pwmMultiplier = 200f;

    [Header("=== Коррекция оси движения ===")]
    [Tooltip("Какая локальная ось модели смотрит вперёд? Например, (0,0,1) = Z, или (1,0,0) = X, или (-1,0,0) = -X")]
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
        
        // Нормализуем вектор направления при старте, чтобы не ломать расчет скорости
        forwardDirection = forwardDirection.normalized;
    }

    public void SetInputs(float gas, float steer)
    {
        gas = Mathf.Clamp(gas, -maxLinearCmd, maxLinearCmd);
        
        // Математика дифференциального привода
        float leftTrackCmd = gas + steer * turnK;
        float rightTrackCmd = gas - steer * turnK;

        float leftPwmTarget = leftTrackCmd * pwmMultiplier;
        float rightPwmTarget = rightTrackCmd * pwmMultiplier;

        leftPwmTarget = ApplyMotorLogic(leftPwmTarget);
        rightPwmTarget = ApplyMotorLogic(rightPwmTarget);

        currentLeftPwm = Mathf.MoveTowards(currentLeftPwm, leftPwmTarget, maxPwmStep);
        currentRightPwm = Mathf.MoveTowards(currentRightPwm, rightPwmTarget, maxPwmStep);
    }

    private float ApplyMotorLogic(float pwm)
    {
        float absPwm = Mathf.Abs(pwm);
        
        if (absPwm < motorDeadzone) 
            return 0f;
            
        return Mathf.Sign(pwm) * Mathf.Max(absPwm, minMotorPwm);
    }

    void FixedUpdate()
    {
        float leftSpeed = currentLeftPwm / pwmMultiplier;
        float rightSpeed = currentRightPwm / pwmMultiplier;

        float linearVelocity = (leftSpeed + rightSpeed) / 2f * moveSpeed;
        float angularVelocity = (leftSpeed - rightSpeed) / 2f * turnSpeed;

        // Переводим локальный вектор коррекции направления в глобальные координаты физического тела
        Vector3 actualForward = rb.rotation * forwardDirection;
        Vector3 moveVector = actualForward * linearVelocity * Time.fixedDeltaTime;
        
        rb.MovePosition(rb.position + moveVector);

        Quaternion turnRotation = Quaternion.Euler(0f, angularVelocity * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }

    void Update()
    {
        float gas = 0f;
        float steer = 0f;

        if (Keyboard.current != null)
        {
            var keyboard = Keyboard.current;

            // Газ (W / S или Стрелки)
            if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) gas = 1f;
            else if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) gas = -1f;

            // Поворот (A / D или Стрелки)
            if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) steer = 1f;
            else if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) steer = -1f;
        }
        
        SetInputs(gas, steer);
    }

    // Отрисовка зеленой стрелочки направления в окне сцены при выделении робота
    void OnDrawGizmosSelected()
    {
        if (forwardDirection != Vector3.zero)
        {
            Gizmos.color = Color.green;
            Vector3 actualForward = transform.TransformDirection(forwardDirection.normalized);
            Gizmos.DrawRay(transform.position, actualForward * 1.5f);
        }
    }
}