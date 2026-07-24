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
    public float usMaxDistance = 3.0f;
    public int usRayCount = 5;
    public float usConeAngle = 30f;
    public string ballTag = "TargetBall";
    public string cubeTag = "TargetCube"; 

    [Header("=== Настройки ИК препятствий ===")]
    [Tooltip("Дальность центрального ИК датчика (метров)")]
    public float centerIRDistance = 0.15f;
    [Tooltip("Дальность левого ИК датчика (метров)")]
    public float leftIRDistance = 0.15f;
    [Tooltip("Дальность правого ИК датчика (метров)")]
    public float rightIRDistance = 0.15f;

    [Header("=== Настройки ИК клешни ===")]
    public float irGripperDistance = 0.08f;

    [Header("=== Выходные данные (Для чтения ROS) ===")]
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
        centerIRObstacle = CheckSingleIR(centerIRPoint, centerIRDistance, Color.orange) ? 1 : 0;
        leftIRObstacle = CheckSingleIR(leftIRPoint, leftIRDistance, Color.orange) ? 1 : 0;
        rightIRObstacle = CheckSingleIR(rightIRPoint, rightIRDistance, Color.orange) ? 1 : 0;
    }
}