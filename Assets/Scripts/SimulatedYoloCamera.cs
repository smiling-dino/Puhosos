using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("=== Настройки YOLO Камеры ===")]
    [Tooltip("Тег объектов, которые камера должна распознавать (например, TargetBall)")]
    public string targetTag = "TargetBall";
    
    [Tooltip("Максимальная дальность видимости (в метрах)")]
    public float maxVisionDistance = 2.0f;

    [Header("=== Выходные данные (Для чтения ИИ) ===")]
    public bool isBallVisible = false;
    
    [Tooltip("Относительное смещение: -1 (крайний левый) .. 0 (центр) .. 1 (крайний правый)")]
    public float horizontalOffset = 0f;
    
    [Tooltip("Нормализованная дистанция: 0 (вплотную) .. 1 (на границе maxVisionDistance)")]
    public float normalizedDistance = 1f;

    private Camera cam;
    private Transform currentTrackedBall = null;

    void Start()
    {
        // Получаем компонент камеры и жестко задаем FOV = 40 градусов
        cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.fieldOfView = 40f;

            // Устанавливаем фиксированное соотношение сторон 4:3 (или 16:9)
            float targetAspect = 4f / 3f; 

            // КРИТИЧЕСКИЙ ФИКС ДЛЯ headless / -nographics режима:
            if (Application.isBatchMode)
            {
                // Задаем проекционную матрицу вручную, так как экрана нет и авто-aspect не сработает
                cam.projectionMatrix = Matrix4x4.Perspective(
                    cam.fieldOfView, 
                    targetAspect, 
                    cam.nearClipPlane, 
                    cam.farClipPlane
                );
            }
            else
            {
                // В режиме с графикой задаем стандартно
                cam.aspect = targetAspect;
            }
        }
    }

    void FixedUpdate()
    {
        ProcessVision();
    }

    /// <summary>
    /// Симуляция YOLO через проекцию 3D-мира на 2D-плоскость камеры
    /// </summary>
    private void ProcessVision()
    {
        Vector3 camPos = transform.position;
        Collider[] collidersInRange = Physics.OverlapSphere(camPos, maxVisionDistance);
        
        Transform bestTarget = null;
        float minDistance = float.MaxValue;
        Vector3 bestViewportPos = Vector3.zero;

        foreach (Collider col in collidersInRange)
        {
            if (col.CompareTag(targetTag))
            {
                Vector3 ballPos = col.transform.position;
                
                // 1. Магия Unity: Переводим 3D координаты в 2D координаты экрана (Viewport: от 0 до 1)
                Vector3 viewportPos = cam.WorldToViewportPoint(ballPos);

                // 2. Проверяем, находится ли мяч перед камерой (z > 0) и в пределах кадра (x и y от 0 до 1)
                if (viewportPos.z > 0 && viewportPos.x >= 0f && viewportPos.x <= 1f && viewportPos.y >= 0f && viewportPos.y <= 1f)
                {
                    Vector3 dirToBall = ballPos - camPos;
                    float actualDistance = dirToBall.magnitude;

                    // 3. Проверка на окклюзию (Line-of-Sight) - видим ли мы мяч, или он за стеной
                    if (Physics.Raycast(camPos, dirToBall.normalized, out RaycastHit hit, maxVisionDistance))
                    {
                        if (hit.collider == col)
                        {
                            if (actualDistance < minDistance)
                            {
                                minDistance = actualDistance;
                                bestTarget = col.transform;
                                bestViewportPos = viewportPos; // Запоминаем позицию на экране для лучшего мяча
                            }
                        }
                    }
                }
            }
        }

        // === РЕЗУЛЬТАТ ОБНАРУЖЕНИЯ ===
        if (bestTarget != null)
        {
            isBallVisible = true;
            normalizedDistance = minDistance / maxVisionDistance;

            // Viewport.x выдает значения от 0 (крайний левый) до 1 (крайний правый)
            // Формула (x - 0.5) * 2 переводит этот диапазон в формат ИИ: от -1 до 1
            horizontalOffset = (bestViewportPos.x - 0.5f) * 2f;
            horizontalOffset = Mathf.Clamp(horizontalOffset, -1f, 1f);
            
            currentTrackedBall = bestTarget;
        }
        else
        {
            ResetVisionState();
        }
    }

    private void ResetVisionState()
    {
        isBallVisible = false;
        horizontalOffset = 0f;
        normalizedDistance = 1f; 
        currentTrackedBall = null;
    }

    void OnDrawGizmos()
    {
        if (isBallVisible && currentTrackedBall != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, currentTrackedBall.position);
        }
    }
}
