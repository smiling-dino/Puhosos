using UnityEngine;

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
    private float brakingEfficiency = 1f;
    private float motorResidualFraction = 0f;
    private float batteryVoltageScale = 1f;
    private int commandDecaySteps = 0;
    private int leftDecayStepsRemaining = 0;
    private int rightDecayStepsRemaining = 0;
    private bool leftStopRequested = false;
    private bool rightStopRequested = false;

    public float MaxLinearCmd => maxLinearCmd;
    public float LeftPwmNormalized => NormalizePwm(currentLeftPwm);
    public float RightPwmNormalized => NormalizePwm(currentRightPwm);
    public float ForwardPwmNormalized => (LeftPwmNormalized + RightPwmNormalized) * 0.5f;
    public float TurnPwmNormalized => (LeftPwmNormalized - RightPwmNormalized) * 0.5f;

    void Awake()
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
        steer = Mathf.Clamp(steer, -1f, 1f);
        
        // Математика дифференциального привода
        float leftTrackCmd = gas + steer * turnK;
        float rightTrackCmd = gas - steer * turnK;

        float leftPwmTarget = leftTrackCmd * pwmMultiplier;
        float rightPwmTarget = rightTrackCmd * pwmMultiplier;

        leftPwmTarget = ApplyMotorLogic(leftPwmTarget);
        rightPwmTarget = ApplyMotorLogic(rightPwmTarget);

        leftPwmTarget = ApplyCommandDecay(
            leftPwmTarget,
            currentLeftPwm,
            ref leftDecayStepsRemaining,
            ref leftStopRequested
        );
        rightPwmTarget = ApplyCommandDecay(
            rightPwmTarget,
            currentRightPwm,
            ref rightDecayStepsRemaining,
            ref rightStopRequested
        );

        float accelerationStep = Mathf.Max(0.01f, maxPwmStep * batteryVoltageScale);
        float brakingStep = accelerationStep * brakingEfficiency;

        currentLeftPwm = Mathf.MoveTowards(
            currentLeftPwm,
            leftPwmTarget,
            IsBraking(currentLeftPwm, leftPwmTarget) ? brakingStep : accelerationStep
        );
        currentRightPwm = Mathf.MoveTowards(
            currentRightPwm,
            rightPwmTarget,
            IsBraking(currentRightPwm, rightPwmTarget) ? brakingStep : accelerationStep
        );
    }

    private float NormalizePwm(float pwm)
    {
        float maxTrackCommand = Mathf.Max(0.001f, maxLinearCmd + Mathf.Abs(turnK));
        float maxReachablePwm = Mathf.Max(1f, maxTrackCommand * pwmMultiplier);
        return Mathf.Clamp(pwm / maxReachablePwm, -1f, 1f);
    }

    private float ApplyMotorLogic(float pwm)
    {
        float absPwm = Mathf.Abs(pwm);
        
        if (absPwm < motorDeadzone) 
            return 0f;
            
        return Mathf.Sign(pwm) * Mathf.Max(absPwm, minMotorPwm);
    }

    private float ApplyCommandDecay(
        float targetPwm,
        float currentPwm,
        ref int decayStepsRemaining,
        ref bool stopRequested)
    {
        if (Mathf.Abs(targetPwm) > 0.001f)
        {
            stopRequested = false;
            decayStepsRemaining = 0;
            return targetPwm;
        }

        if (!stopRequested)
        {
            stopRequested = true;
            decayStepsRemaining = commandDecaySteps;
        }

        if (decayStepsRemaining <= 0 || Mathf.Abs(currentPwm) <= 0.001f)
        {
            return 0f;
        }

        decayStepsRemaining--;
        return currentPwm * motorResidualFraction;
    }

    private static bool IsBraking(float currentPwm, float targetPwm)
    {
        return Mathf.Abs(targetPwm) < Mathf.Abs(currentPwm)
            || (Mathf.Abs(currentPwm) > 0.001f && Mathf.Sign(targetPwm) != Mathf.Sign(currentPwm));
    }

    void FixedUpdate()
    {
        float leftSpeed = currentLeftPwm / pwmMultiplier;
        float rightSpeed = currentRightPwm / pwmMultiplier;

        float linearVelocity = (leftSpeed + rightSpeed) / 2f * moveSpeed * batteryVoltageScale;
        float angularVelocity = (leftSpeed - rightSpeed) / 2f * turnSpeed * batteryVoltageScale;

        // Переводим локальный вектор коррекции направления в глобальные координаты физического тела
        Vector3 actualForward = rb.rotation * forwardDirection;
        Vector3 moveVector = actualForward * linearVelocity * Time.fixedDeltaTime;
        
        rb.MovePosition(rb.position + moveVector);

        Quaternion turnRotation = Quaternion.Euler(0f, angularVelocity * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }

    public void ResetMotors()
    {
        currentLeftPwm = 0f;
        currentRightPwm = 0f;
        leftDecayStepsRemaining = 0;
        rightDecayStepsRemaining = 0;
        leftStopRequested = false;
        rightStopRequested = false;
    }

    public void ConfigureEpisodeDynamics(
        float episodeBrakingEfficiency,
        int episodeCommandDecaySteps,
        float episodeMotorResidualFraction,
        float episodeBatteryVoltageScale)
    {
        brakingEfficiency = Mathf.Clamp(episodeBrakingEfficiency, 0.1f, 3f);
        commandDecaySteps = Mathf.Clamp(episodeCommandDecaySteps, 0, 20);
        motorResidualFraction = Mathf.Clamp(episodeMotorResidualFraction, 0f, 0.25f);
        batteryVoltageScale = Mathf.Clamp(episodeBatteryVoltageScale, 0.5f, 1.5f);
        ResetMotors();
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
