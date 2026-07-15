using UnityEngine;

public class VirtualSensors : MonoBehaviour
{
    [Header("=== Точки-якоря датчиков ===")]
    [Tooltip("Центральный УЗ датчик (вперед)")]
    public Transform centerPoint;
    [Tooltip("Центральный ИК датчик (вперед)")]
    public Transform centerIRPoint;
    [Tooltip("Левый ИК датчик (влево)")]
    public Transform leftIRPoint;
    [Tooltip("Правый ИК датчик (вправо)")]
    public Transform rightIRPoint;
    [Tooltip("ИК датчик внутри клешни")]
    public Transform gripperIRPoint;

    [Header("=== Настройки Ультразвука ===")]
    [Tooltip("Максимальная дистанция УЗ датчика (метров)")]
    public float usMaxDistance = 2.0f;
    [Tooltip("Количество лучей в конусе обзора")]
    public int usRayCount = 5;
    [Tooltip("Угол конуса обзора в градусах")]
    public float usConeAngle = 30f;
    [Tooltip("Тег мяча, который УЗ датчик должен игнорировать")]
    public string ballTag = "TargetBall";

    [Header("=== Настройки ИК препятствий ===")]
    [Tooltip("Дальность ИК датчиков препятствий (15 см = 0.15f)")]
    public float irObstacleDistance = 0.15f;

    [Header("=== Настройки ИК клешни ===")]
    [Tooltip("Дальность ИК датчика внутри клешни (7-8 см = 0.08f)")]
    public float irGripperDistance = 0.08f;

    [Header("=== Выходные данные (Для чтения другими скриптами) ===")]
    [Range(0f, 1f)] public float ultrasoundValue = 1f; // 0 - вплотную, 1 - чисто
    public int centerIRObstacle = 0;  // 1 - стена, 0 - пусто
    public int leftIRObstacle = 0;    // 1 - стена, 0 - пусто
    public int rightIRObstacle = 0;   // 1 - стена, 0 - пусто
    public bool isBallInGripper = false;

    void FixedUpdate()
    {
        UpdateUltrasound();
        UpdateObstacleIR();
        UpdateGripperIR();
    }

    /// <summary>
    /// Симуляция УЗ-датчика веером лучей с игнорированием мяча
    /// </summary>
    private void UpdateUltrasound()
    {
        if (centerPoint == null) return;

        float minDistance = usMaxDistance;
        Vector3 origin = centerPoint.position;
        
        // Считаем шаг угла между лучами веера
        float angleStep = usRayCount > 1 ? usConeAngle / (usRayCount - 1) : 0f;
        float startAngle = -usConeAngle / 2f;

        // --- ШАГ 1: ФИЗИКА (Находим общий минимум) ---
        for (int i = 0; i < usRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            Quaternion rotation = Quaternion.AngleAxis(currentAngle, centerPoint.up);
            Vector3 direction = rotation * centerPoint.forward;

            // Пускаем луч
            RaycastHit[] hits = Physics.RaycastAll(origin, direction, usMaxDistance);
            
            // Ищем самый близкий объект (игнорируя мяч)
            foreach (var hit in hits)
            {
                if (hit.collider.CompareTag(ballTag)) 
                    continue; // Игнорируем мяч

                if (hit.distance < minDistance)
                {
                    minDistance = hit.distance;
                }
            }
        }

        // --- ШАГ 2: ВИЗУАЛИЗАЦИЯ (Отрисовываем ВСЕ лучи с учетом найденного минимума) ---
        Color rayColor = minDistance < usMaxDistance ? Color.red : Color.cyan;

        for (int i = 0; i < usRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            Quaternion rotation = Quaternion.AngleAxis(currentAngle, centerPoint.up);
            Vector3 direction = rotation * centerPoint.forward;

            // Теперь все лучи гарантированно рисуются одинаковой минимальной длины!
            Debug.DrawRay(origin, direction * minDistance, rayColor);
        }

        // Нормализуем значение: 0 (вплотную) до 1 (чисто)
        ultrasoundValue = minDistance / usMaxDistance;
    }

    /// <summary>
    /// Симуляция коротких ИК датчиков препятствий вокруг робота
    /// </summary>
    private void UpdateObstacleIR()
    {
        centerIRObstacle = CheckSingleIR(centerIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        leftIRObstacle = CheckSingleIR(leftIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        rightIRObstacle = CheckSingleIR(rightIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
    }

    /// <summary>
    /// Симуляция датчика внутри клешни на обнаружение конкретно мяча
    /// </summary>
    private void UpdateGripperIR()
    {
        if (gripperIRPoint == null) return;

        Vector3 origin = gripperIRPoint.position;
        Vector3 direction = gripperIRPoint.forward;

        // Пускаем одиночный луч из клешни
        if (Physics.Raycast(origin, direction, out RaycastHit hit, irGripperDistance))
        {
            // Проверяем, что пойман именно целевой мяч
            if (hit.collider.CompareTag(ballTag))
            {
                isBallInGripper = true;
                Debug.DrawRay(origin, direction * hit.distance, Color.magenta);
                return;
            }
        }

        isBallInGripper = false;
        Debug.DrawRay(origin, direction * irGripperDistance, Color.gray);
    }

    /// <summary>
    /// Универсальный метод для проверки одиночного ИК луча
    /// </summary>
    private bool CheckSingleIR(Transform point, float distance, Color debugColor)
    {
        if (point == null) return false;

        Vector3 origin = point.position;
        Vector3 direction = point.forward;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, distance))
        {
            Debug.DrawRay(origin, direction * hit.distance, Color.red);
            return true; // Наткнулись на стену/препятствие
        }

        Debug.DrawRay(origin, direction * distance, debugColor);
        return false; // Путь чист
    }
}