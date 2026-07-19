using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("=== Точка захвата ===")]
    [Tooltip("Точка между губками клешни")]
    [SerializeField] private Transform holdPoint;

    [Header("=== Подъём всей клешни ===")]
    [Tooltip("Circle.001 — корень всей подвижной клешни")]
    [SerializeField] private Transform liftPivot;

    [Tooltip("Локальная ось вращения Circle.001")]
    [SerializeField] private Vector3 liftLocalAxis = Vector3.forward;

    [Tooltip("Угол полностью поднятой клешни")]
    [SerializeField] private float raisedAngleDegrees = 50f;

    [Tooltip("Скорость подъёма: долей полного хода в секунду")]
    [SerializeField, Min(0.01f)] private float liftNormalizedSpeed = 0.6f;

    [Header("=== Открытие и закрытие ===")]
    [SerializeField] private Transform leftClawPivot;
    [SerializeField] private Transform rightClawPivot;

    [Tooltip("Локальная ось вращения левой стороны")]
    [SerializeField] private Vector3 leftClawLocalAxis = Vector3.up;

    [Tooltip("Локальная ось вращения правой стороны")]
    [SerializeField] private Vector3 rightClawLocalAxis = Vector3.up;

    [SerializeField] private float leftClosedAngleDegrees = 20f;
    [SerializeField] private float rightClosedAngleDegrees = -20f;

    [Tooltip("Скорость закрытия: долей полного хода в секунду")]
    [SerializeField, Min(0.01f)] private float gripNormalizedSpeed = 3f;

    [Header("=== Текущее состояние ===")]
    [SerializeField, Range(0f, 1f)] private float liftNormalized;
    [SerializeField, Range(0f, 1f)] private float gripNormalized;
    [SerializeField] private bool targetClosed;

    private Quaternion liftBaseRotation;
    private Quaternion leftBaseRotation;
    private Quaternion rightBaseRotation;
    private bool defaultPoseCaptured;
    private float liftInput;

    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;
    private Transform originalBallParent;

    public bool IsClosed => targetClosed;

    public bool HasGripActuator =>
        leftClawPivot != null && rightClawPivot != null;

    public bool IsFullyClosed =>
        HasGripActuator
        && targetClosed
        && gripNormalized >= 0.999f;

    public bool IsFullyOpen =>
        !targetClosed && gripNormalized <= 0.001f;

    public bool HasLiftActuator => liftPivot != null;

    public float LiftNormalized => liftNormalized;
    public float GripNormalized => gripNormalized;

    public bool IsAtLiftTop => liftNormalized >= 0.999f;
    public bool IsAtLiftBottom => liftNormalized <= 0.001f;

    private void Awake()
    {
        CaptureDefaultPose();
        ResetMechanism();

        if (holdPoint == null)
        {
            Debug.LogError("HoldPoint не назначен в GripperController", this);
        }

        if (liftPivot == null)
        {
            Debug.LogError("Lift Pivot не назначен в GripperController", this);
        }

        if (!HasGripActuator)
        {
            Debug.LogError(
                "LeftClawPivot или RightClawPivot не назначен в GripperController",
                this
            );
        }
    }

    private void FixedUpdate()
    {
        float gripTarget = targetClosed ? 1f : 0f;

        gripNormalized = Mathf.MoveTowards(
            gripNormalized,
            gripTarget,
            gripNormalizedSpeed * Time.fixedDeltaTime
        );

        if (liftInput > 0f)
        {
            liftNormalized = Mathf.MoveTowards(
                liftNormalized,
                1f,
                liftNormalizedSpeed * Time.fixedDeltaTime
            );
        }
        else if (liftInput < 0f)
        {
            liftNormalized = Mathf.MoveTowards(
                liftNormalized,
                0f,
                liftNormalizedSpeed * Time.fixedDeltaTime
            );
        }

        ApplyPose();
    }

    public void SetClosed(bool closed)
    {
        targetClosed = closed;
    }

    public void SetLiftAction(int actionId)
    {
        if (!HasLiftActuator)
        {
            liftInput = 0f;
            return;
        }

        liftInput = actionId == 1
            ? 1f
            : actionId == 2
                ? -1f
                : 0f;
    }

    public void ResetLift()
    {
        liftInput = 0f;
        liftNormalized = 0f;
        ApplyPose();
    }

    public void ResetMechanism()
    {
        if (!defaultPoseCaptured)
        {
            CaptureDefaultPose();
        }

        liftInput = 0f;
        liftNormalized = 0f;
        gripNormalized = 0f;
        targetClosed = false;

        ApplyPose();
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

        heldBall.transform.SetParent(holdPoint, false);
        heldBall.transform.localPosition = Vector3.zero;
        heldBall.transform.localRotation = Quaternion.identity;
    }

    public void ReleaseBall()
    {
        liftInput = 0f;

        if (heldBall == null)
        {
            return;
        }

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
            Vector3 closestPoint =
                targetCollider.ClosestPoint(holdPoint.position);

            return Vector3.Distance(
                holdPoint.position,
                closestPoint
            );
        }

        return Vector3.Distance(
            holdPoint.position,
            ball.transform.position
        );
    }

    private void CaptureDefaultPose()
    {
        if (defaultPoseCaptured)
        {
            return;
        }

        if (liftPivot != null)
        {
            liftBaseRotation = liftPivot.localRotation;
        }

        if (leftClawPivot != null)
        {
            leftBaseRotation = leftClawPivot.localRotation;
        }

        if (rightClawPivot != null)
        {
            rightBaseRotation = rightClawPivot.localRotation;
        }

        defaultPoseCaptured = true;
    }

    private void ApplyPose()
    {
        if (!defaultPoseCaptured)
        {
            return;
        }

        if (liftPivot != null)
        {
            liftPivot.localRotation =
                GetAxisRotation(
                    liftLocalAxis,
                    raisedAngleDegrees * liftNormalized
                )
                * liftBaseRotation;
        }

        if (leftClawPivot != null)
        {
            leftClawPivot.localRotation =
                leftBaseRotation
                * GetAxisRotation(
                    leftClawLocalAxis,
                    leftClosedAngleDegrees * gripNormalized
                );
        }

        if (rightClawPivot != null)
        {
            rightClawPivot.localRotation =
                rightBaseRotation
                * GetAxisRotation(
                    rightClawLocalAxis,
                    rightClosedAngleDegrees * gripNormalized
                );
        }
    }

    private static Quaternion GetAxisRotation(
        Vector3 axis,
        float angleDegrees
    )
    {
        if (axis.sqrMagnitude < 0.000001f)
        {
            return Quaternion.identity;
        }

        return Quaternion.AngleAxis(
            angleDegrees,
            axis.normalized
        );
    }
}