using UnityEngine;
using UnityEngine.InputSystem; // Добавили пространство имен новой системы ввода

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

    [Header("=== Коррекция оси движения ===")]
    [Tooltip("Какая локальная ось модели смотрит вперёд? Обычно (0,0,1) = Z, но может быть (1,0,0) = X")]
    public Vector3 forwardDirection = Vector3.forward; 

    // Приватные поля
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

    public void SetInputs(float gas, float steer)
    {
        gas = Mathf.Clamp(gas, -maxLinearCmd, maxLinearCmd);
        float leftTrackCmd = gas - steer * turnK;
        float rightTrackCmd = gas + steer * turnK;

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
        float angularVelocity = (rightSpeed - leftSpeed) / 2f * turnSpeed;

        Vector3 actualForward = transform.TransformDirection(forwardDirection);

        Vector3 moveVector = actualForward * linearVelocity * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + moveVector);

        Quaternion turnRotation = Quaternion.Euler(0f, angularVelocity * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }

    void Update()
    {
        float gas = 0f;
        float steer = 0f;

        // Опрашиваем клавиатуру через новый Input System
        if (Keyboard.current != null)
        {
            var keyboard = Keyboard.current;

            // Газ (W / S или Стрелки вверх / вниз)
            if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) gas = 1f;
            else if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) gas = -1f;

            // Руль (A / D или Стрелки влево / вправо)
            if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) steer = 1f;
            else if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) steer = -1f;
        }
        
        // Передаем значения в логику моторов
        SetInputs(gas, steer);
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
}