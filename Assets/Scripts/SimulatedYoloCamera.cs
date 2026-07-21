using System.Collections.Generic;
using UnityEngine;

// Класс для хранения настроек и результатов поиска одной цели
[System.Serializable]
public class YoloTargetInfo
{
    [Tooltip("Тег искомого объекта (например, TargetBall)")]
    public string targetTag;
    [Tooltip("Цвет луча в редакторе (OnDrawGizmos)")]
    public Color gizmoColor = Color.green;
    
    [Header("Выходные данные (Read-only)")]
    public bool isVisible = false;
    public float horizontalOffset = 0f;
    public float normalizedDistance = 1f;
    
    [HideInInspector] 
    public Transform currentTrackedObject = null;
}

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("=== Настройки YOLO Камеры ===")]
    [Tooltip("Список объектов, которые камера должна распознавать")]
    public YoloTargetInfo[] targets;
    
    [Tooltip("Максимальная дальность видимости (в метрах)")]
    public float maxVisionDistance = 2.0f;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.fieldOfView = 40f;
            float targetAspect = 4f / 3f; 

            if (Application.isBatchMode)
            {
                cam.projectionMatrix = Matrix4x4.Perspective(
                    cam.fieldOfView, 
                    targetAspect, 
                    cam.nearClipPlane, 
                    cam.farClipPlane
                );
            }
            else
            {
                cam.aspect = targetAspect;
            }
        }
    }

    void FixedUpdate()
    {
        ProcessVision();
    }

    private void ProcessVision()
    {
        Vector3 camPos = transform.position;
        Collider[] collidersInRange = Physics.OverlapSphere(camPos, maxVisionDistance);
        
        // Перебираем каждую настроенную цель (мяч, куб и т.д.)
        foreach (var target in targets)
        {
            // Сброс состояния перед новым кадром
            target.isVisible = false;
            target.horizontalOffset = 0f;
            target.normalizedDistance = 1f;
            target.currentTrackedObject = null;

            Transform bestTarget = null;
            float minDistance = float.MaxValue;
            Vector3 bestViewportPos = Vector3.zero;

            foreach (Collider col in collidersInRange)
            {
                if (col.CompareTag(target.targetTag))
                {
                    Vector3 objPos = col.transform.position;
                    Vector3 viewportPos = cam.WorldToViewportPoint(objPos);

                    if (viewportPos.z > 0 && viewportPos.x >= 0f && viewportPos.x <= 1f && viewportPos.y >= 0f && viewportPos.y <= 1f)
                    {
                        Vector3 dirToObj = objPos - camPos;
                        float actualDistance = dirToObj.magnitude;

                        // Добавлено игнорирование триггеров, чтобы зоны не перекрывали зрение
                        if (Physics.Raycast(camPos, dirToObj.normalized, out RaycastHit hit, maxVisionDistance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
                        {
                            if (hit.collider == col)
                            {
                                if (actualDistance < minDistance)
                                // Ищем самый близкий объект данного тега
                                {
                                    minDistance = actualDistance;
                                    bestTarget = col.transform;
                                    bestViewportPos = viewportPos;
                                }
                            }
                        }
                    }
                }
            }

            // Сохраняем результаты для конкретной цели
            if (bestTarget != null)
            {
                target.isVisible = true;
                target.normalizedDistance = minDistance / maxVisionDistance;
                target.horizontalOffset = Mathf.Clamp((bestViewportPos.x - 0.5f) * 2f, -1f, 1f);
                target.currentTrackedObject = bestTarget;
            }
        }
    }

    void OnDrawGizmos()
    {
        if (targets == null) return;

        foreach (var target in targets)
        {
            if (target.isVisible && target.currentTrackedObject != null)
            {
                Gizmos.color = target.gizmoColor;
                Gizmos.DrawLine(transform.position, target.currentTrackedObject.position);
            }
        }
    }
    
    /// <summary>
    /// Вспомогательный метод для получения данных конкретной цели из других скриптов
    /// </summary>
    public YoloTargetInfo GetTarget(string tagToFind)
    {
        if (targets == null) return null;
        foreach (var target in targets)
        {
            if (target.targetTag == tagToFind)
                return target;
        }
        return null;
    }
}