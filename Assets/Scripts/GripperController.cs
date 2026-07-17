using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("=== Настройки захвата ===")]
    [Tooltip("Точка между губками клешни")]
    public Transform holdPoint;

    [Header("=== Подъём клешни ===")]
    [Tooltip("Подвижная часть клешни. Для упрощённой симуляции можно назначить HoldPoint")]
    public Transform liftRoot;
    [Min(0.01f)] public float maxLiftHeightMeters = 0.15f;
    [Min(0.01f)] public float liftSpeedMetersPerSecond = 0.10f;

    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;
    private Vector3 liftBaseLocalPosition;
    private float liftHeightMeters;
    private float liftInput;
    private bool liftDefaultsCaptured;
    private bool isClosed;

    // Родитель мяча до захвата — TrainingArea конкретного агента
    private Transform originalBallParent;

    public bool IsClosed => isClosed;
    public bool HasLiftActuator => liftRoot != null;
    public float LiftHeightMeters => liftHeightMeters;
    public float LiftNormalized => maxLiftHeightMeters > 0f
        ? Mathf.Clamp01(liftHeightMeters / maxLiftHeightMeters)
        : 0f;
    public float LiftInput => liftInput;
    public bool IsAtLiftBottom => liftHeightMeters <= 0.0001f;
    public bool IsAtLiftTop => liftHeightMeters >= maxLiftHeightMeters - 0.0001f;

    private void Awake()
    {
        CaptureLiftDefaults();
        ApplyLiftPose();
    }

    private void FixedUpdate()
    {
        if (!HasLiftActuator || Mathf.Approximately(liftInput, 0f))
        {
            return;
        }

        float targetHeight = liftInput > 0f ? maxLiftHeightMeters : 0f;
        liftHeightMeters = Mathf.MoveTowards(
            liftHeightMeters,
            targetHeight,
            liftSpeedMetersPerSecond * Time.fixedDeltaTime
        );
        ApplyLiftPose();
    }

    public void SetClosed(bool closed)
    {
        isClosed = closed;
    }

    public void SetLiftAction(int actionId)
    {
        if (!HasLiftActuator)
        {
            liftInput = 0f;
            return;
        }

        liftInput = actionId == 1 ? 1f : actionId == 2 ? -1f : 0f;
    }

    public void ResetLift()
    {
        CaptureLiftDefaults();
        liftInput = 0f;
        liftHeightMeters = 0f;
        ApplyLiftPose();
    }

    public void GrabBall(GameObject ball)
    {
        if (heldBall != null || ball == null)
        {
            return;
        }

        if (holdPoint == null)
        {
            Debug.LogError("HoldPoint не назначен в GripperController", this);
            return;
        }

        heldBall = ball;
        ballRigidbody = heldBall.GetComponent<Rigidbody>();
        ballCollider = heldBall.GetComponent<Collider>();

        // Запоминаем арену, которой принадлежит мяч
        originalBallParent = heldBall.transform.parent;

        if (ballRigidbody != null)
        {
            ballRigidbody.linearVelocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
            ballRigidbody.isKinematic = true;
        }

        if (ballCollider != null)
        {
            ballCollider.enabled = false;
        }

        // Прикрепляем мяч к клешне
        heldBall.transform.SetParent(holdPoint, false);
        heldBall.transform.localPosition = Vector3.zero;
        heldBall.transform.localRotation = Quaternion.identity;
    }

    public bool IsBallInsideCaptureZone(GameObject ball, float captureRadius)
    {
        return GetDistanceToHoldPoint(ball) <= captureRadius;
    }

    public float GetDistanceToHoldPoint(GameObject ball)
    {
        if (holdPoint == null || ball == null)
        {
            return float.PositiveInfinity;
        }

        Collider targetCollider = ball.GetComponent<Collider>();
        if (targetCollider != null && targetCollider.enabled)
        {
            Vector3 closestPoint = targetCollider.ClosestPoint(holdPoint.position);
            return Vector3.Distance(holdPoint.position, closestPoint);
        }

        return Vector3.Distance(holdPoint.position, ball.transform.position);
    }

    public void ReleaseBall()
    {
        liftInput = 0f;

        if (heldBall == null)
        {
            return;
        }

        // Возвращаем мяч обратно в его TrainingArea
        Transform targetParent = originalBallParent != null
            ? originalBallParent
            : transform.parent;

        heldBall.transform.SetParent(targetParent, true);

        if (ballCollider != null)
        {
            ballCollider.enabled = true;
        }

        if (ballRigidbody != null)
        {
            ballRigidbody.isKinematic = false;
            ballRigidbody.linearVelocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
        }

        heldBall = null;
        ballRigidbody = null;
        ballCollider = null;
        originalBallParent = null;
    }

    public bool IsHoldingBall()
    {
        return heldBall != null;
    }

    private void CaptureLiftDefaults()
    {
        if (liftDefaultsCaptured || liftRoot == null)
        {
            return;
        }

        liftBaseLocalPosition = liftRoot.localPosition;
        liftDefaultsCaptured = true;
    }

    private void ApplyLiftPose()
    {
        if (!liftDefaultsCaptured || liftRoot == null)
        {
            return;
        }

        Transform parent = liftRoot.parent;
        float parentUpScale = parent != null
            ? parent.TransformVector(Vector3.up).magnitude
            : 1f;
        float localHeight = liftHeightMeters / Mathf.Max(0.0001f, parentUpScale);
        liftRoot.localPosition = liftBaseLocalPosition + Vector3.up * localHeight;
    }
}
