using System.Collections.Generic;
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
    [Tooltip("Корень робота, коллайдеры которого датчики должны игнорировать")]
    public Transform robotRoot;

    [Header("=== Фильтрация физики ===")]
    public LayerMask obstacleLayers = ~0;
    public LayerMask gripperDetectionLayers = ~0;

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

    public float UltrasoundDistanceMeters => ultrasoundValue * usMaxDistance;

    private float rawUltrasoundValue = 1f;
    private int rawCenterIRObstacle = 0;
    private int rawLeftIRObstacle = 0;
    private int rawRightIRObstacle = 0;
    private bool rawBallInGripper = false;

    private bool noiseEnabled = false;
    private float ultrasoundScale = 1f;
    private float ultrasoundBiasMeters = 0f;
    private float ultrasoundNoiseMeters = 0f;
    private float ultrasoundQuantizationMeters = 0f;
    private float ultrasoundDropoutProbability = 0f;
    private int ultrasoundLatencySteps = 0;
    private float irFalsePositiveProbability = 0f;
    private float irFalseNegativeProbability = 0f;
    private int irLatencySteps = 0;

    private readonly Queue<float> ultrasoundLatencyBuffer = new Queue<float>();
    private readonly Queue<int> centerIrLatencyBuffer = new Queue<int>();
    private readonly Queue<int> leftIrLatencyBuffer = new Queue<int>();
    private readonly Queue<int> rightIrLatencyBuffer = new Queue<int>();
    private readonly Queue<bool> gripperIrLatencyBuffer = new Queue<bool>();

    private void Awake()
    {
        if (robotRoot == null)
        {
            robotRoot = transform;
        }
    }

    void FixedUpdate()
    {
        UpdateUltrasound();
        UpdateObstacleIR();
        UpdateGripperIR();
        PublishSensorReadings();
    }

    /// <summary>
    /// Симуляция УЗ-датчика веером лучей с игнорированием мяча
    /// </summary>
    private void UpdateUltrasound()
    {
        rawUltrasoundValue = 1f;
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
            RaycastHit[] hits = Physics.RaycastAll(
                origin,
                direction,
                usMaxDistance,
                obstacleLayers,
                QueryTriggerInteraction.Ignore
            );
            
            // Ищем самый близкий объект (игнорируя мяч)
            foreach (var hit in hits)
            {
                if (IsSelfCollider(hit.collider) || hit.collider.CompareTag(ballTag))
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
        rawUltrasoundValue = minDistance / Mathf.Max(0.001f, usMaxDistance);
    }

    /// <summary>
    /// Симуляция коротких ИК датчиков препятствий вокруг робота
    /// </summary>
    private void UpdateObstacleIR()
    {
        rawCenterIRObstacle = CheckSingleIR(centerIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        rawLeftIRObstacle = CheckSingleIR(leftIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        rawRightIRObstacle = CheckSingleIR(rightIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
    }

    /// <summary>
    /// Симуляция датчика внутри клешни на обнаружение конкретно мяча
    /// </summary>
    private void UpdateGripperIR()
    {
        rawBallInGripper = false;
        if (gripperIRPoint == null) return;

        Vector3 origin = gripperIRPoint.position;
        Vector3 direction = gripperIRPoint.forward;

        // Пускаем одиночный луч из клешни
        RaycastHit[] hits = Physics.RaycastAll(
            origin,
            direction,
            irGripperDistance,
            gripperDetectionLayers,
            QueryTriggerInteraction.Ignore
        );

        float closestBallDistance = float.PositiveInfinity;
        foreach (RaycastHit hit in hits)
        {
            if (IsSelfCollider(hit.collider) || !hit.collider.CompareTag(ballTag))
            {
                continue;
            }

            closestBallDistance = Mathf.Min(closestBallDistance, hit.distance);
        }

        if (!float.IsPositiveInfinity(closestBallDistance))
        {
            rawBallInGripper = true;
            Debug.DrawRay(origin, direction * closestBallDistance, Color.magenta);
            return;
        }

        Debug.DrawRay(origin, direction * irGripperDistance, Color.gray);
    }

    private void PublishSensorReadings()
    {
        float measuredDistance = Mathf.Clamp01(rawUltrasoundValue) * Mathf.Max(0.001f, usMaxDistance);
        if (noiseEnabled)
        {
            if (Random.value < ultrasoundDropoutProbability)
            {
                measuredDistance = usMaxDistance;
            }
            else
            {
                measuredDistance = measuredDistance * ultrasoundScale
                    + ultrasoundBiasMeters
                    + Random.Range(-ultrasoundNoiseMeters, ultrasoundNoiseMeters);

                if (ultrasoundQuantizationMeters > 0.0001f)
                {
                    measuredDistance = Mathf.Round(measuredDistance / ultrasoundQuantizationMeters)
                        * ultrasoundQuantizationMeters;
                }
            }
        }

        float measuredUltrasound = Mathf.Clamp01(measuredDistance / Mathf.Max(0.001f, usMaxDistance));
        int measuredCenterIr = ApplyIrNoise(rawCenterIRObstacle != 0) ? 1 : 0;
        int measuredLeftIr = ApplyIrNoise(rawLeftIRObstacle != 0) ? 1 : 0;
        int measuredRightIr = ApplyIrNoise(rawRightIRObstacle != 0) ? 1 : 0;
        bool measuredGripperIr = ApplyIrNoise(rawBallInGripper);

        ultrasoundValue = GetDelayedValue(
            ultrasoundLatencyBuffer,
            measuredUltrasound,
            noiseEnabled ? ultrasoundLatencySteps : 0,
            1f
        );
        centerIRObstacle = GetDelayedValue(
            centerIrLatencyBuffer,
            measuredCenterIr,
            noiseEnabled ? irLatencySteps : 0,
            0
        );
        leftIRObstacle = GetDelayedValue(
            leftIrLatencyBuffer,
            measuredLeftIr,
            noiseEnabled ? irLatencySteps : 0,
            0
        );
        rightIRObstacle = GetDelayedValue(
            rightIrLatencyBuffer,
            measuredRightIr,
            noiseEnabled ? irLatencySteps : 0,
            0
        );
        isBallInGripper = GetDelayedValue(
            gripperIrLatencyBuffer,
            measuredGripperIr,
            noiseEnabled ? irLatencySteps : 0,
            false
        );
    }

    private bool ApplyIrNoise(bool rawValue)
    {
        if (!noiseEnabled)
        {
            return rawValue;
        }

        if (rawValue)
        {
            return Random.value >= irFalseNegativeProbability;
        }

        return Random.value < irFalsePositiveProbability;
    }

    private static T GetDelayedValue<T>(
        Queue<T> buffer,
        T currentValue,
        int latencySteps,
        T initialValue)
    {
        if (latencySteps <= 0)
        {
            buffer.Clear();
            return currentValue;
        }

        buffer.Enqueue(currentValue);
        if (buffer.Count <= latencySteps)
        {
            return initialValue;
        }

        return buffer.Dequeue();
    }

    public void ConfigureEpisodeNoise(
        bool enabled,
        float episodeUltrasoundScale,
        float episodeUltrasoundBiasMeters,
        float episodeUltrasoundNoiseMeters,
        float episodeUltrasoundQuantizationMeters,
        float episodeUltrasoundDropoutProbability,
        int episodeUltrasoundLatencySteps,
        float episodeIrFalsePositiveProbability,
        float episodeIrFalseNegativeProbability,
        int episodeIrLatencySteps)
    {
        noiseEnabled = enabled;
        ultrasoundScale = enabled ? Mathf.Clamp(episodeUltrasoundScale, 0.5f, 1.5f) : 1f;
        ultrasoundBiasMeters = enabled ? Mathf.Clamp(episodeUltrasoundBiasMeters, -0.25f, 0.25f) : 0f;
        ultrasoundNoiseMeters = enabled ? Mathf.Max(0f, episodeUltrasoundNoiseMeters) : 0f;
        ultrasoundQuantizationMeters = enabled ? Mathf.Max(0f, episodeUltrasoundQuantizationMeters) : 0f;
        ultrasoundDropoutProbability = enabled
            ? Mathf.Clamp01(episodeUltrasoundDropoutProbability)
            : 0f;
        ultrasoundLatencySteps = enabled ? Mathf.Max(0, episodeUltrasoundLatencySteps) : 0;
        irFalsePositiveProbability = enabled
            ? Mathf.Clamp01(episodeIrFalsePositiveProbability)
            : 0f;
        irFalseNegativeProbability = enabled
            ? Mathf.Clamp01(episodeIrFalseNegativeProbability)
            : 0f;
        irLatencySteps = enabled ? Mathf.Max(0, episodeIrLatencySteps) : 0;

        ClearLatencyBuffers();
        ultrasoundValue = 1f;
        centerIRObstacle = 0;
        leftIRObstacle = 0;
        rightIRObstacle = 0;
        isBallInGripper = false;
    }

    private void ClearLatencyBuffers()
    {
        ultrasoundLatencyBuffer.Clear();
        centerIrLatencyBuffer.Clear();
        leftIrLatencyBuffer.Clear();
        rightIrLatencyBuffer.Clear();
        gripperIrLatencyBuffer.Clear();
    }

    /// <summary>
    /// Универсальный метод для проверки одиночного ИК луча
    /// </summary>
    private bool CheckSingleIR(Transform point, float distance, Color debugColor)
    {
        if (point == null) return false;

        Vector3 origin = point.position;
        Vector3 direction = point.forward;

        RaycastHit[] hits = Physics.RaycastAll(
            origin,
            direction,
            distance,
            obstacleLayers,
            QueryTriggerInteraction.Ignore
        );

        float closestDistance = float.PositiveInfinity;
        foreach (RaycastHit hit in hits)
        {
            if (IsSelfCollider(hit.collider) || hit.collider.CompareTag(ballTag))
            {
                continue;
            }

            closestDistance = Mathf.Min(closestDistance, hit.distance);
        }

        if (!float.IsPositiveInfinity(closestDistance))
        {
            Debug.DrawRay(origin, direction * closestDistance, Color.red);
            return true;
        }

        Debug.DrawRay(origin, direction * distance, debugColor);
        return false; // Путь чист
    }

    private bool IsSelfCollider(Collider candidate)
    {
        return candidate != null
            && robotRoot != null
            && (candidate.transform == robotRoot || candidate.transform.IsChildOf(robotRoot));
    }
}
