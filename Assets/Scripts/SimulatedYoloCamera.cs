using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("=== Настройки YOLO Камеры ===")]
    [Tooltip("Ссылка на целевой мяч")]
    public Transform targetBall;
    
    [Tooltip("Максимальная дальность видимости (в метрах)")]
    public float maxVisionDistance = 2.0f;
    
    [Tooltip("Горизонтальный угол обзора (hFOV) в градусах")]
    public float hFOV = 40f;

    [Header("=== Выходные данные (Для чтения ИИ) ===")]
    public bool isBallVisible = false;
    
    [Tooltip("Относительное смещение: -1 (крайний левый) .. 0 (центр) .. 1 (крайний правый)")]
    public float horizontalOffset = 0f;
    
    [Tooltip("Нормализованная дистанция: 0 (вплотную) .. 1 (на границе maxVisionDistance)")]
    public float normalizedDistance = 1f;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
    }

    void FixedUpdate()
    {
        ProcessVision();
    }

    /// <summary>
    /// Математическая симуляция обнаружения мяча (YOLO)
    /// </summary>
    private void ProcessVision()
    {
        // Базовая защита от отсутствия цели
        if (targetBall == null)
        {
            ResetVisionState();
            return;
        }

        Vector3 camPos = transform.position;
        Vector3 ballPos = targetBall.position;
        Vector3 dirToBall = ballPos - camPos;
        float actualDistance = dirToBall.magnitude;

        // 1. Проверка дистанции (мяч слишком далеко?)
        if (actualDistance > maxVisionDistance)
        {
            ResetVisionState();
            return;
        }

        // 2. Проверка угла обзора (Попал ли мяч в hFOV = 40°)
        // Угол между направлением камеры (вперед) и направлением на мяч
        float angleToBall = Vector3.Angle(transform.forward, dirToBall);
        if (angleToBall > hFOV / 2f) // Делим на 2, так как конус расходится в обе стороны (±20°)
        {
            ResetVisionState();
            return;
        }

        // 3. Проверка препятствий (Line-of-Sight через Raycast)
        if (Physics.Raycast(camPos, dirToBall.normalized, out RaycastHit hit, maxVisionDistance))
        {
            // Если луч наткнулся на что-то, и это НЕ мяч — значит между ними стена
            if (hit.transform != targetBall)
            {
                ResetVisionState();
                return;
            }
        }
        else
        {
            // Луч прошел мимо всего (защита от багов физики)
            ResetVisionState();
            return;
        }

        // === МЯЧ УСПЕШНО ОБНАРУЖЕН ===
        isBallVisible = true;

        // Расчет нормализованной дистанции (0 до 1)
        normalizedDistance = actualDistance / maxVisionDistance;

        // Расчет 2D-координат на экране (Проекция)
        Vector3 viewportPoint = cam.WorldToViewportPoint(ballPos);

        // Viewport.x выдает от 0 (лево) до 1 (право). 
        // Формула (x - 0.5) * 2 переводит это в диапазон [-1; 1], где 0 — идеальный центр.
        horizontalOffset = (viewportPoint.x - 0.5f) * 2f;
        horizontalOffset = Mathf.Clamp(horizontalOffset, -1f, 1f); // Защита от выхода за границы
    }

    /// <summary>
    /// Сброс значений, если мяч потерян из виду
    /// </summary>
    private void ResetVisionState()
    {
        isBallVisible = false;
        horizontalOffset = 0f;
        normalizedDistance = 1f; // 1 означает, что мяч максимально далеко / недоступен
    }

    /// <summary>
    /// Отрисовка луча камеры в редакторе для удобства отладки
    /// </summary>
    void OnDrawGizmos()
    {
        if (isBallVisible && targetBall != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, targetBall.position);
        }
    }
}