using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target Settings")]
    [SerializeField] private Transform targetBall;          // Ссылка на объект мяча
    [SerializeField] private Transform robotRoot;           // Корень робота для поиска корпуса камеры
    [SerializeField] private Collider cameraHostCollider;   // Единственный коллайдер, внутри которого находится камера
    [SerializeField] private LayerMask obstacleLayers = ~0; // Слои препятствий (стены, коробки)

    [Header("Camera Specs")]
    [SerializeField] private float maxDetectionDistance = 2.0f; // Дальность видимости камеры
    [SerializeField] private float horizontalFov = 40f;         // Угол обзора по горизонтали

    private Camera cam;

    public float MaxDetectionDistance => maxDetectionDistance;
    public float HorizontalFov => horizontalFov;

    void Awake()
    {
        cam = GetComponent<Camera>();

        if (robotRoot == null)
        {
            TrackController controller = GetComponentInParent<TrackController>();
            robotRoot = controller != null ? controller.transform : transform.parent;
        }

        ResolveCameraHostCollider();
    }

    /// <summary>
    /// Проверяет, видит ли "YOLO" мяч прямо сейчас с учетом FOV, дистанции и преград
    /// </summary>
    public bool IsBallVisible()
    {
        if (targetBall == null) return false;
        if (!IsFinite(targetBall.position) || !IsFinite(transform.position)) return false;

        // 1. Проверка дистанции
        float distance = Vector3.Distance(transform.position, targetBall.position);
        if (float.IsNaN(distance) || float.IsInfinity(distance)) return false;
        if (distance > maxDetectionDistance) return false;

        // 2. Проверка угла обзора (FOV)
        Vector3 directionToBall = (targetBall.position - transform.position).normalized;
        float angle = Mathf.Abs(GetSignedHorizontalAngleToBall());
        if (float.IsNaN(angle) || float.IsInfinity(angle)) return false;
        if (angle > (horizontalFov / 2f)) return false;

        // 3. Проверка преград. Игнорируются только корпус, внутри которого стоит камера,
        // и целевой мяч. Все остальные коллайдеры, включая части робота, перекрывают обзор.
        RaycastHit[] hits = Physics.RaycastAll(
            transform.position,
            directionToBall,
            distance,
            obstacleLayers,
            QueryTriggerInteraction.Ignore
        );

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider == cameraHostCollider || IsTargetCollider(hit.transform))
            {
                continue;
            }

            return false;
        }

        return true;
    }

    public float GetHorizontalAngleToBall()
    {
        return Mathf.Abs(GetSignedHorizontalAngleToBall());
    }

    public float GetSignedHorizontalAngleToBall()
    {
        if (targetBall == null) return 180f;
        if (!IsFinite(targetBall.position) || !IsFinite(transform.position)) return 180f;

        Vector3 flatForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
        Vector3 flatDirectionToBall = Vector3.ProjectOnPlane(targetBall.position - transform.position, Vector3.up);

        if (flatForward.sqrMagnitude < 0.0001f || flatDirectionToBall.sqrMagnitude < 0.0001f)
        {
            return 180f;
        }

        return Vector3.SignedAngle(
            flatForward.normalized,
            flatDirectionToBall.normalized,
            Vector3.up
        );
    }

    /// <summary>
    /// Возвращает относительный угол до мяча: от -1 (слева) до 1 (справа). 0 — строго по центру.
    /// </summary>
    public float GetNormalizedHorizontalAngle()
    {
        if (!IsBallVisible()) return 0f;

        Vector3 viewportPos = cam.WorldToViewportPoint(targetBall.position);
        // Преобразуем диапазон [0, 1] в [-1, 1]
        return Mathf.Clamp((viewportPos.x - 0.5f) * 2f, -1f, 1f);
    }

    /// <summary>
    /// Возвращает нормализованную дистанцию от 0 (в упор) до 1 (на границе видимости).
    /// </summary>
    public float GetNormalizedDistance()
    {
        if (!IsBallVisible() || targetBall == null) return 1f;

        float distance = GetDistanceToBall();
        return Mathf.Clamp01(distance / maxDetectionDistance);
    }

    public float GetDistanceToBall()
    {
        if (targetBall == null) return float.PositiveInfinity;
        if (!IsFinite(targetBall.position) || !IsFinite(transform.position)) return float.PositiveInfinity;

        return Vector3.Distance(transform.position, targetBall.position);
    }

    public Transform GetBallTransform()
    {
        return targetBall;
    }

    private bool IsFinite(Vector3 value)
    {
        return !float.IsNaN(value.x) && !float.IsInfinity(value.x)
            && !float.IsNaN(value.y) && !float.IsInfinity(value.y)
            && !float.IsNaN(value.z) && !float.IsInfinity(value.z);
    }

    private void ResolveCameraHostCollider()
    {
        if (cameraHostCollider != null || robotRoot == null)
        {
            return;
        }

        // Ищем только среди коллайдеров объектов-родителей камеры. Коллайдеры клешней,
        // креплений и других соседних частей робота не могут стать исключением случайно.
        Transform current = transform.parent;
        while (current != null)
        {
            Collider[] colliders = current.GetComponents<Collider>();
            foreach (Collider candidate in colliders)
            {
                if (candidate != null && candidate.enabled)
                {
                    Vector3 closestPoint = candidate.ClosestPoint(transform.position);
                    if ((closestPoint - transform.position).sqrMagnitude <= 0.000001f)
                    {
                        cameraHostCollider = candidate;
                        return;
                    }
                }
            }

            if (current == robotRoot)
            {
                break;
            }

            current = current.parent;
        }
    }

    private bool IsTargetCollider(Transform candidate)
    {
        return candidate != null
            && targetBall != null
            && (candidate == targetBall || candidate.IsChildOf(targetBall));
    }

    public void SetDetectionProfile(float detectionDistance, float fov)
    {
        maxDetectionDistance = Mathf.Max(0.1f, detectionDistance);
        horizontalFov = Mathf.Clamp(fov, 10f, 120f);
    }
}
