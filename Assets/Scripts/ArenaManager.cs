using System.Collections.Generic;
using UnityEngine;

public class ArenaManager : MonoBehaviour
{
    [Header("=== Объекты арены ===")]
    public GameObject targetBall;
    public GameObject targetCube;
    public GameObject obstaclePrefab;
    
    [Header("=== Зоны спавна (BoxCollider как триггер) ===")]
    public BoxCollider obstacleZone;
    public BoxCollider targetZone;
    public BoxCollider homeZone;

    [Header("=== Настройки спавна ===")]
    public int obstacleCount = 5;
    [Tooltip("Максимальное количество попыток найти свободное место")]
    public int maxSpawnAttempts = 30;

    // Размеры коробки для проверки коллизий (половина размера для Physics.CheckBox)
    private Vector3 obstacleExtents = new Vector3(0.075f, 0.075f, 0.4f); 
    private float ballRadius = 0.05f; // Укажи радиус твоего шарика
    private Vector3 cubeExtents = new Vector3(0.025f, 0.025f, 0.025f);

    private List<GameObject> spawnedObstacles = new List<GameObject>();

    void Start()
    {
        // Создаем пул препятствий один раз при старте
        for (int i = 0; i < obstacleCount; i++)
        {
            GameObject obs = Instantiate(obstaclePrefab, transform);
            obs.SetActive(false);
            spawnedObstacles.Add(obs);
        }

        ResetArena();
    }

    

    /// <summary>
    /// Вызывается агентом при начале нового эпизода
    /// </summary>
    public void ResetArena()
    {
        PlaceObstacles();
        PlaceBall();
        PlaceCube();
    }

    private void PlaceObstacles()
    {
        foreach (var obs in spawnedObstacles)
        {
            obs.SetActive(false); 
            
            Vector3 finalPos = Vector3.zero;
            bool validPositionFound = false;

            for (int i = 0; i < maxSpawnAttempts; i++)
            {
                Vector3 randomPos = GetRandomPointInBox(obstacleZone);
                
                // 1. Узнаем точную высоту пола (нижняя граница obstacleZone)
                float floorY = obstacleZone.bounds.min.y;

                // 2. Позиция для ПРОВЕРКИ (поднимаем ровно настолько, чтобы не задеть пол)
                Vector3 checkPos = randomPos;
                checkPos.y = floorY + obstacleExtents.y + 0.02f; // +2см безопасный зазор

                if (!Physics.CheckBox(checkPos, obstacleExtents, Quaternion.identity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
                {
                    validPositionFound = true;
                    
                    // 3. Позиция для УСТАНОВКИ
                    // Если префаб сделан из стандартного куба Unity, его центр ровно посередине.
                    // Значит, чтобы он стоял на полу, его центр нужно поднять на половину высоты (0.37 / 2 = 0.185)
                    finalPos = randomPos;
                    finalPos.y = floorY + obstacleExtents.y; 
                    
                    break;
                }
            }

            if (validPositionFound)
            {
                obs.transform.position = finalPos;
                obs.transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
                obs.SetActive(true);
                
                Rigidbody rb = obs.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
            else
            {
                Debug.LogWarning("Не удалось найти свободное место для препятствия!");
            }
        }
    }
    
    private void PlaceBall()
    {
        if (targetBall == null) return;

        Vector3 finalPos = Vector3.zero;
        bool validPositionFound = false;

        // 1. Узнаем точную высоту пола (нижняя граница targetZone)
        float floorY = targetZone.bounds.min.y;

        for (int i = 0; i < maxSpawnAttempts; i++)
        {
            Vector3 randomPos = GetRandomPointInBox(targetZone);
            
            // 2. Позиция для ПРОВЕРКИ (чуть приподнимаем сферу, чтобы не задеть пол)
            Vector3 checkPos = randomPos;
            checkPos.y = floorY + ballRadius + 0.02f; // +2см безопасный зазор

            // КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: Добавляем QueryTriggerInteraction.Ignore,
            // чтобы сфера проверки не билась о саму зону TargetZone
            if (!Physics.CheckSphere(checkPos, ballRadius, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
            {
                validPositionFound = true;
                
                // 3. Позиция для УСТАНОВКИ (ровно на пол с учетом радиуса)
                finalPos = randomPos;
                finalPos.y = floorY + ballRadius;
                break;
            }
        }

        if (validPositionFound)
        {
            targetBall.transform.position = finalPos;
            
            Rigidbody rb = targetBall.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            Debug.LogWarning("Не удалось найти свободное место для мяча! Проверь размеры TargetZone.");
        }
    }

    private void PlaceCube()
    {
        // Если кубик или зона не назначены в инспекторе — выходим
        if (targetCube == null || homeZone == null) return;

        Vector3 finalPos = Vector3.zero;
        bool validPositionFound = false;

        // 1. Узнаем точную высоту пола (нижняя граница homeZone)
        float floorY = homeZone.bounds.min.y;

        for (int i = 0; i < maxSpawnAttempts; i++) //[cite: 2]
        {
            Vector3 randomPos = GetRandomPointInBox(homeZone); // Используем твой метод генерации[cite: 2]
            
            // 2. Позиция для ПРОВЕРКИ (поднимаем кубик, чтобы не задеть пол)
            Vector3 checkPos = randomPos;
            checkPos.y = floorY + cubeExtents.y + 0.02f; // +2см безопасный зазор

            // Проверяем коллизии, игнорируя триггеры
            if (!Physics.CheckBox(checkPos, cubeExtents, Quaternion.identity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
            {
                validPositionFound = true;
                
                // 3. Позиция для УСТАНОВКИ (ровно на пол с учетом высоты кубика)
                finalPos = randomPos;
                finalPos.y = floorY + cubeExtents.y;
                break;
            }
        }

        if (validPositionFound)
        {
            targetCube.transform.position = finalPos;
            
            // Задаем случайное вращение кубику по оси Y, если это нужно
            targetCube.transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
            
            // Сбрасываем физику, чтобы он не улетел от старых сил
            Rigidbody rb = targetCube.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            Debug.LogWarning("Не удалось найти свободное место для кубика! Проверь размеры HomeZone.");
        }
    }

    /// <summary>
    /// Генерирует случайную точку внутри BoxCollider
    /// </summary>
    private Vector3 GetRandomPointInBox(BoxCollider box)
    {
        Vector3 extents = box.size / 2f;
        Vector3 point = new Vector3(
            Random.Range(-extents.x, extents.x),
            Random.Range(-extents.y, extents.y),
            Random.Range(-extents.z, extents.z)
        );

        // Переводим локальные координаты коллайдера в мировые
        return box.transform.TransformPoint(point);
    }
}