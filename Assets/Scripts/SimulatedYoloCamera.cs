using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target Settings")]
    [SerializeField] private Transform targetBall;          // Ссылка на объект мяча
    [SerializeField] private LayerMask obstacleLayers;      // Слои препятствий (стены, коробки)

    [Header("Camera Specs")]
    [SerializeField] private float maxDetectionDistance = 2.0f; // Дальность видимости камеры
    [SerializeField] private float horizontalFov = 40f;         // Угол обзора по горизонтали

    private Camera cam;

    public float MaxDetectionDistance => maxDetectionDistance;
    public float HorizontalFov => horizontalFov;

    void Awake()
    {
        cam = GetComponent<Camera>();
    }

    /// <summary>
    /// Проверяет, видит ли "YOLO" мяч прямо сейчас с учетом FOV, дистанции и преград
    /// </summary>
    public bool IsBallVisible()
    {
        if (targetBall == null) return false;

        // 1. Проверка дистанции
        float distance = Vector3.Distance(transform.position, targetBall.position);
        if (distance > maxDetectionDistance) return false;

        // 2. Проверка угла обзора (FOV)
        Vector3 directionToBall = (targetBall.position - transform.position).normalized;
        float angle = Vector3.Angle(transform.forward, directionToBall);
        if (angle > (horizontalFov / 2f)) return false;

        // 3. Проверка преград (Raycast)
        RaycastHit hit;
        if (Physics.Raycast(transform.position, directionToBall, out hit, distance, obstacleLayers))
        {
            // Если луч попал во что-то, что не является мячом (или его родителем), видимость перекрыта
            if (hit.transform != targetBall && !hit.transform.IsChildOf(targetBall))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Возвращает относительный угол до мяча: от -1 (слева) до 1 (справа). 0 — строго по центру.
    /// </summary>
    public float GetNormalizedHorizontalAngle()
    {
        if (!IsBallVisible()) return 0f;

        Vector3 viewportPos = cam.WorldToViewportPoint(targetBall.position);
        // Преобразуем диапазон [0, 1] в [-1, 1]
        return (viewportPos.x - 0.5f) * 2f;
    }

    /// <summary>
    /// Возвращает нормализованную дистанцию от 0 (в упор) до 1 (на границе видимости).
    /// </summary>
    public float GetNormalizedDistance()
    {
        if (!IsBallVisible() || targetBall == null) return 1f;

        float distance = Vector3.Distance(transform.position, targetBall.position);
        return Mathf.Clamp01(distance / maxDetectionDistance);
    }

    public Transform GetBallTransform()
    {
        return targetBall;
    }

    public void SetDetectionProfile(float detectionDistance, float fov)
    {
        maxDetectionDistance = Mathf.Max(0.1f, detectionDistance);
        horizontalFov = Mathf.Clamp(fov, 10f, 120f);
    }
}
