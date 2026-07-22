using UnityEngine;
using UnityEngine.Serialization;

public class GripperController : MonoBehaviour
{
    [Header("=== Точка захвата ===")]
    [Tooltip("Точка между губками клешни")]
    [SerializeField] private Transform holdPoint;

    [Header("=== Складывание руки буквой Z ===")]
    [Tooltip("Шарнир руки у основания робота")]
    [SerializeField] private Transform liftPivot;

    [Tooltip("Второй шарнир руки")]
    [SerializeField] private Transform elbowPivot;

    [Tooltip("Автоматически искать LiftServoPivot и Cube003ServoPivot по именам")]
    [SerializeField] private bool findPivotsByName = true;

    [Tooltip("Автоматически определять ось первого шарнира")]
    [SerializeField] private bool autoDetectLiftAxis = true;

    [Tooltip("Ось первого шарнира в координатах его родителя")]
    [FormerlySerializedAs("liftLocalAxis")]
    [SerializeField] private Vector3 liftAxisInParentSpace =
        Vector3.forward;

    [Tooltip("Автоматически определять ось второго шарнира")]
    [SerializeField] private bool autoDetectElbowAxis = true;

    [Tooltip("Ось второго шарнира в координатах его родителя")]
    [SerializeField] private Vector3 elbowAxisInParentSpace =
        Vector3.forward;

    [Tooltip("Поворот нижнего звена вверх по диагонали")]
    [FormerlySerializedAs("raisedAngleDegrees")]
    [SerializeField, Range(-20f, 160f)]
    private float shoulderFoldAngleDegrees = 160f;

    [Tooltip("Противоположный поворот второго звена")]
    [SerializeField, Range(-180f, 90f)]
    private float elbowFoldAngleDegrees = -180f;

    [Tooltip("Скорость складывания руки")]
    [SerializeField, Min(0.01f)]
    private float liftNormalizedSpeed = 0.6f;

    [Header("=== Открытие и закрытие клешни ===")]
    [SerializeField] private Transform leftClawPivot;
    [SerializeField] private Transform rightClawPivot;

    [Tooltip("Локальная ось левой части клешни")]
    [SerializeField] private Vector3 leftClawLocalAxis =
        Vector3.up;

    [Tooltip("Локальная ось правой части клешни")]
    [SerializeField] private Vector3 rightClawLocalAxis =
        Vector3.up;

    [SerializeField] private float leftClosedAngleDegrees = 20f;
    [SerializeField] private float rightClosedAngleDegrees = -20f;

    [Tooltip("Скорость закрытия клешни")]
    [SerializeField, Min(0.01f)]
    private float gripNormalizedSpeed = 3f;

    [Header("=== Текущее состояние ===")]
    [SerializeField, Range(0f, 1f)]
    private float liftNormalized;

    [SerializeField, Range(0f, 1f)]
    private float gripNormalized;

    [SerializeField]
    private bool targetClosed;

    private Quaternion liftBaseRotation;
    private Quaternion elbowBaseRotation;
    private Quaternion leftBaseRotation;
    private Quaternion rightBaseRotation;

    private Vector3 resolvedLiftAxisInParentSpace =
        Vector3.forward;

    private Vector3 resolvedElbowAxisInParentSpace =
        Vector3.forward;

    private bool defaultPoseCaptured;
    private float liftInput;

    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;
    private Transform originalBallParent;

    public bool IsClosed => targetClosed;

    public bool HasGripActuator =>
        leftClawPivot != null &&
        rightClawPivot != null;

    public bool IsFullyClosed =>
        HasGripActuator &&
        targetClosed &&
        gripNormalized >= 0.999f;

    public bool IsFullyOpen =>
        !targetClosed &&
        gripNormalized <= 0.001f;

    /*
     * Для складывания обязательно нужны оба шарнира.
     * Если Cube003ServoPivot не назначен, старый одиночный
     * подъём выполняться не будет.
     */
    public bool HasFoldActuator =>
        liftPivot != null &&
        elbowPivot != null;

    // Совместимость с RobotBrain.
    public bool HasLiftActuator => HasFoldActuator;

    public float FoldNormalized => liftNormalized;
    public float LiftNormalized => liftNormalized;
    public float GripNormalized => gripNormalized;

    public bool IsAtFoldedPose =>
        liftNormalized >= 0.999f;

    public bool IsAtExtendedPose =>
        liftNormalized <= 0.001f;

    // Совместимость со старым RobotBrain.
    public bool IsAtLiftTop =>
        IsAtFoldedPose;

    public bool IsAtLiftBottom =>
        IsAtExtendedPose;

    private void Awake()
    {
        /*
         * Сначала ищем новые pivot-объекты.
         * Только затем запоминаем исходные повороты.
         */
        ResolvePivotReferences();
        CaptureDefaultPose();
        ResetMechanism();

        ValidateConfiguration();
    }

    private void OnValidate()
    {
        /*
         * Позволяет Unity автоматически заполнить ссылки
         * ещё до запуска сцены.
         */
        ResolvePivotReferences();
    }

    private void FixedUpdate()
    {
        UpdateGrip();
        UpdateArmFold();
        ApplyPose();
    }

    private void UpdateGrip()
    {
        float gripTarget = targetClosed ? 1f : 0f;

        gripNormalized = Mathf.MoveTowards(
            gripNormalized,
            gripTarget,
            gripNormalizedSpeed * Time.fixedDeltaTime
        );
    }

    private void UpdateArmFold()
    {
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
    }

    public void SetClosed(bool closed)
    {
        targetClosed = closed;
    }

    /*
     * actionId:
     * 0 — остановить руку;
     * 1 — складывать буквой Z;
     * 2 — раскладывать.
     */
    public void SetFoldAction(int actionId)
    {
        if (!HasFoldActuator)
        {
            liftInput = 0f;
            return;
        }

        if (actionId == 1)
        {
            liftInput = 1f;
        }
        else if (actionId == 2)
        {
            liftInput = -1f;
        }
        else
        {
            liftInput = 0f;
        }
    }

    /*
     * Старое имя оставлено для RobotBrain.
     * Теперь оно запускает складывание двух шарниров.
     */
    public void SetLiftAction(int actionId)
    {
        SetFoldAction(actionId);
    }

    public void ResetFold()
    {
        liftInput = 0f;
        liftNormalized = 0f;
        ApplyPose();
    }

    public void ResetLift()
    {
        ResetFold();
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
            Debug.LogError(
                "HoldPoint не назначен в GripperController",
                this
            );
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

        Transform targetParent =
            originalBallParent != null
                ? originalBallParent
                : transform.parent;

        heldBall.transform.SetParent(
            targetParent,
            true
        );

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

    public bool IsBallInsideCaptureZone(
        GameObject ball,
        float captureRadius
    )
    {
        return GetDistanceToHoldPoint(ball)
            <= captureRadius;
    }

    public float GetDistanceToHoldPoint(GameObject ball)
    {
        if (holdPoint == null || ball == null)
        {
            return float.PositiveInfinity;
        }

        Collider targetCollider =
            ball.GetComponent<Collider>();

        if (targetCollider != null &&
            targetCollider.enabled)
        {
            Vector3 closestPoint =
                targetCollider.ClosestPoint(
                    holdPoint.position
                );

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
            liftBaseRotation =
                liftPivot.localRotation;

            resolvedLiftAxisInParentSpace =
                ResolveAxisInParentSpace(
                    liftPivot,
                    autoDetectLiftAxis,
                    liftAxisInParentSpace,
                    "LiftServoPivot"
                );
        }

        if (elbowPivot != null)
        {
            elbowBaseRotation =
                elbowPivot.localRotation;

            resolvedElbowAxisInParentSpace =
                ResolveAxisInParentSpace(
                    elbowPivot,
                    autoDetectElbowAxis,
                    elbowAxisInParentSpace,
                    "Cube003ServoPivot"
                );
        }

        if (leftClawPivot != null)
        {
            leftBaseRotation =
                leftClawPivot.localRotation;
        }

        if (rightClawPivot != null)
        {
            rightBaseRotation =
                rightClawPivot.localRotation;
        }

        defaultPoseCaptured = true;
    }

    private void ApplyPose()
    {
        if (!defaultPoseCaptured)
        {
            return;
        }

        /*
         * Первый шарнир поднимает нижнее звено
         * вверх по диагонали.
         */
        if (liftPivot != null)
        {
            Quaternion shoulderRotation =
                GetAxisRotation(
                    resolvedLiftAxisInParentSpace,
                    shoulderFoldAngleDegrees *
                    liftNormalized
                );

            liftPivot.localRotation =
                shoulderRotation *
                liftBaseRotation;
        }

        /*
         * Второй шарнир одновременно вращается
         * в противоположную сторону.
         *
         * 60° и -60° взаимно компенсируются,
         * поэтому верхнее звено остаётся почти
         * горизонтальным — получается буква Z.
         */
        if (elbowPivot != null)
        {
            Quaternion elbowRotation =
                GetAxisRotation(
                    resolvedElbowAxisInParentSpace,
                    elbowFoldAngleDegrees *
                    liftNormalized
                );

            elbowPivot.localRotation =
                elbowRotation *
                elbowBaseRotation;
        }

        if (leftClawPivot != null)
        {
            leftClawPivot.localRotation =
                leftBaseRotation *
                GetAxisRotation(
                    leftClawLocalAxis,
                    leftClosedAngleDegrees *
                    gripNormalized
                );
        }

        if (rightClawPivot != null)
        {
            rightClawPivot.localRotation =
                rightBaseRotation *
                GetAxisRotation(
                    rightClawLocalAxis,
                    rightClosedAngleDegrees *
                    gripNormalized
                );
        }
    }

    private Vector3 ResolveAxisInParentSpace(
        Transform pivot,
        bool autoDetectAxis,
        Vector3 configuredAxis,
        string pivotName
    )
    {
        Vector3 fallbackAxis =
            configuredAxis.sqrMagnitude > 0.000001f
                ? configuredAxis.normalized
                : Vector3.forward;

        if (!autoDetectAxis ||
            pivot == null ||
            holdPoint == null)
        {
            return fallbackAxis;
        }

        /*
         * Направление от шарнира к клешне.
         */
        Vector3 pivotToHoldPoint =
            holdPoint.position -
            pivot.position;

        /*
         * Ось, перпендикулярная плоскости,
         * образованной рукой и вертикалью робота.
         *
         * Благодаря этому рука поднимается вверх,
         * а не поворачивается влево или вправо.
         */
        Vector3 worldFoldAxis =
            Vector3.Cross(
                pivotToHoldPoint,
                transform.up
            );

        if (worldFoldAxis.sqrMagnitude <
            0.000001f)
        {
            Debug.LogWarning(
                "Не удалось определить ось " +
                pivotName +
                ": шарнир и HoldPoint находятся " +
                "на одной вертикали",
                this
            );

            return fallbackAxis;
        }

        worldFoldAxis.Normalize();

        /*
         * В ApplyPose вращение задаётся в координатах
         * родителя pivot-объекта.
         */
        if (pivot.parent != null)
        {
            return pivot.parent
                .InverseTransformDirection(
                    worldFoldAxis
                )
                .normalized;
        }

        return worldFoldAxis;
    }

    private void ResolvePivotReferences()
    {
        if (!findPivotsByName)
        {
            return;
        }

        Transform namedLiftPivot =
            FindDescendantByName(
                "LiftServoPivot"
            );

        Transform namedElbowPivot =
            FindDescendantByName(
                "Cube003ServoPivot"
            );

        if (namedLiftPivot != null)
        {
            liftPivot = namedLiftPivot;
        }

        if (namedElbowPivot != null)
        {
            elbowPivot = namedElbowPivot;
        }
    }

    private Transform FindDescendantByName(
        string objectName
    )
    {
        Transform[] descendants =
            GetComponentsInChildren<Transform>(
                true
            );

        foreach (Transform descendant in descendants)
        {
            if (descendant.name == objectName)
            {
                return descendant;
            }
        }

        return null;
    }

    private void ValidateConfiguration()
    {
        if (holdPoint == null)
        {
            Debug.LogError(
                "HoldPoint не назначен",
                this
            );
        }

        if (liftPivot == null)
        {
            Debug.LogError(
                "LiftServoPivot не назначен",
                this
            );
        }

        if (elbowPivot == null)
        {
            Debug.LogError(
                "Cube003ServoPivot не назначен. " +
                "Без него рука не сможет " +
                "складываться буквой Z.",
                this
            );
        }

        if (liftPivot != null &&
            elbowPivot != null &&
            !elbowPivot.IsChildOf(liftPivot))
        {
            Debug.LogError(
                "Cube003ServoPivot должен быть " +
                "дочерним объектом LiftServoPivot",
                this
            );
        }

        if (holdPoint != null &&
            elbowPivot != null &&
            !holdPoint.IsChildOf(elbowPivot))
        {
            Debug.LogError(
                "HoldPoint должен находиться ниже " +
                "Cube003ServoPivot в иерархии",
                this
            );
        }

        if (!HasGripActuator)
        {
            Debug.LogError(
                "LeftClawPivot или RightClawPivot " +
                "не назначен",
                this
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