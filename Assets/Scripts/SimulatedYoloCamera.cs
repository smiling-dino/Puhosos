using UnityEngine;

public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("=== Настройки YOLO Камеры (Характеристики железа) ===")]
    [Tooltip("Тег объектов, которые камера должна распознавать (например, TargetBall)")]
    public string targetTag = "TargetBall";
    
    [Tooltip("Максимальная дальность видимости (в метрах)")]
    public float maxVisionDistance = 2.0f;
    
    [Tooltip("Горизонтальный угол обзора (hFOV) в градусах")]
    public float hFOV = 60f; // Укажите реальные данные объектива
    
    [Tooltip("Вертикальный угол обзора (vFOV) в градусах")]
    public float vFOV = 40f; // Укажите реальные данные объектива

    [Header("=== Выходные данные (Для чтения ИИ) ===")]
    public bool isBallVisible = false;
    
    [Tooltip("Относительное смещение: -1 (крайний левый) .. 0 (центр) .. 1 (крайний правый)")]
    public float horizontalOffset = 0f;
    
    [Tooltip("Нормализованная дистанция: 0 (вплотную) .. 1 (на границе maxVisionDistance)")]
    public float normalizedDistance = 1f;

    // Переменная чисто для отрисовки луча в редакторе (Gizmos)
    private Transform currentTrackedBall = null;

    void FixedUpdate()
    {
        ProcessVision();
    }

    /// <summary>
    /// Математическая симуляция прямоугольного FOV аппаратной камеры
    /// </summary>
    private void ProcessVision()
    {
        Vector3 camPos = transform.position;
        Collider[] collidersInRange = Physics.OverlapSphere(camPos, maxVisionDistance);
        
        Transform bestTarget = null;
        float minDistance = float.MaxValue;
        float bestSignedHAngle = 0f; // Запомним угол лучшего мяча для расчета Offset

        foreach (Collider col in collidersInRange)
        {
            if (col.CompareTag(targetTag))
            {
                Vector3 ballPos = col.transform.position;
                Vector3 dirToBall = ballPos - camPos;
                float actualDistance = dirToBall.magnitude;

                // 1. Переводим позицию мяча в локальные координаты камеры
                // Теперь Z - это расстояние прямо от объектива, X - вправо/влево, Y - вверх/вниз
                Vector3 localPos = transform.InverseTransformPoint(ballPos);

                // Если мяч сзади камеры (отрицательный Z), сразу отбрасываем
                if (localPos.z <= 0f) continue;

                // 2. Вычисляем углы отклонения от центра объектива
                // Используем тригонометрию (Atan2) для получения угла по осям
                float hAngleAbs = Mathf.Atan2(Mathf.Abs(localPos.x), localPos.z) * Mathf.Rad2Deg;
                float vAngleAbs = Mathf.Atan2(Mathf.Abs(localPos.y), localPos.z) * Mathf.Rad2Deg;

                // 3. Проверяем попадание в "прямоугольник" FOV
                if (hAngleAbs > hFOV / 2f || vAngleAbs > vFOV / 2f)
                {
                    continue; // Мяч за пределами кадра по горизонтали или вертикали
                }

                // 4. Проверка препятствий (Line-of-Sight)
                if (Physics.Raycast(camPos, dirToBall.normalized, out RaycastHit hit, maxVisionDistance))
                {
                    if (hit.collider == col)
                    {
                        if (actualDistance < minDistance)
                        {
                            minDistance = actualDistance;
                            bestTarget = col.transform;
                            // Вычисляем угол со знаком (влево -, вправо +) для нейросети
                            bestSignedHAngle = Mathf.Atan2(localPos.x, localPos.z) * Mathf.Rad2Deg;
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

            // Расчет горизонтального смещения от -1 до 1 напрямую из угла
            // Если угол равен половине hFOV, получится 1 или -1.
            horizontalOffset = bestSignedHAngle / (hFOV / 2f);
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