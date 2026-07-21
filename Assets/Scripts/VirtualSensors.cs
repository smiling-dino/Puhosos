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
    public float usMaxDistance = 3.0f;
    [Tooltip("Количество лучей в конусе обзора")]
    public int usRayCount = 5;
    [Tooltip("Угол конуса обзора в градусах")]
    public float usConeAngle = 30f;
    [Tooltip("Тег мяча, который УЗ датчик должен игнорировать")]
    public string ballTag = "TargetBall";
    [Tooltip("Тег куба (зоны выгрузки), который УЗ датчик должен игнорировать")]
    public string cubeTag = "TargetCube"; // <--- ДОБАВЛЕНО

    [Header("=== Настройки ИК препятствий ===")]
    [Tooltip("Дальность ИК датчиков препятствий (15 см = 0.15f)")]
    public float irObstacleDistance = 0.15f;

    [Header("=== Настройки ИК клешни ===")]
    [Tooltip("Дальность ИК датчика внутри клешни (7-8 см = 0.08f)")]
    public float irGripperDistance = 0.08f;

    [Header("=== Выходные данные (Для чтения другими скриптами) ===")]
    [Range(0f, 1f)] public float ultrasoundValue = 1f; 
    public int centerIRObstacle = 0;  
    public int leftIRObstacle = 0;    
    public int rightIRObstacle = 0;   
    public bool isBallInGripper = false;

    void FixedUpdate()
    {
        UpdateUltrasound();
        UpdateObstacleIR();
        UpdateGripperIR();
    }

    private void UpdateUltrasound()
    {
        if (centerPoint == null) return;

        float minDistance = usMaxDistance;
        Vector3 origin = centerPoint.position;
        float angleStep = usRayCount > 1 ? usConeAngle / (usRayCount - 1) : 0f;
        float startAngle = -usConeAngle / 2f;

        for (int i = 0; i < usRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            Quaternion rotation = Quaternion.AngleAxis(currentAngle, centerPoint.up);
            Vector3 direction = rotation * centerPoint.forward;

            RaycastHit[] hits = Physics.RaycastAll(origin, direction, usMaxDistance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
            
            foreach (var hit in hits)
            {
                // КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: Игнорируем и мяч, и куб
                if (hit.collider.CompareTag(ballTag) || hit.collider.CompareTag(cubeTag)) 
                    continue; 

                if (hit.distance < minDistance)
                {
                    minDistance = hit.distance;
                }
            }
        }

        Color rayColor = minDistance < usMaxDistance ? Color.red : Color.cyan;
        for (int i = 0; i < usRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            Quaternion rotation = Quaternion.AngleAxis(currentAngle, centerPoint.up);
            Vector3 direction = rotation * centerPoint.forward;
            Debug.DrawRay(origin, direction * minDistance, rayColor);
        }

        ultrasoundValue = minDistance / usMaxDistance;
    }

    private void UpdateGripperIR()
    {
        if (gripperIRPoint == null) return;

        Vector3 origin = gripperIRPoint.position;
        Vector3 direction = gripperIRPoint.forward;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, irGripperDistance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Collide))
        {
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

    private bool CheckSingleIR(Transform point, float distance, Color debugColor)
    {
        if (point == null) return false;

        Vector3 origin = point.position;
        Vector3 direction = point.forward;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, distance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            Debug.DrawRay(origin, direction * hit.distance, Color.red);
            return true; 
        }

        Debug.DrawRay(origin, direction * distance, debugColor);
        return false; 
    }

    private void UpdateObstacleIR()
    {
        centerIRObstacle = CheckSingleIR(centerIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        leftIRObstacle = CheckSingleIR(leftIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
        rightIRObstacle = CheckSingleIR(rightIRPoint, irObstacleDistance, Color.orange) ? 1 : 0;
    }
}