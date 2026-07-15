using System.Collections.Generic;
using UnityEngine;

public class ArenaController : MonoBehaviour
{
    [Header("Настройки спавна")]
    public GameObject obstaclePrefab; // Префаб препятствия
    public Transform robotTransform;   // Ссылка на трансформ робота
    public Transform targetTransform;  // Ссылка на трансформ цели

    [Header("Привязка к полу")]
    [Tooltip("Перетащи сюда объект Plane (пол арены), который является потомком TrainingArea")]
    public MeshRenderer arenaFloor;    // Рендерер плоскости для расчета реальных границ
    
    [Tooltip("Отступ от краев пола (в метрах), чтобы объекты не спавнились на самой границе")]
    public float edgePadding = 0.5f;

    public int numberOfObstacles = 5;  // Сколько препятствий генерировать
    public float safeRadius = 1.5f;    // Радиус безопасности вокруг робота и цели

    [Header("Настройки высоты (Локальная ось Y)")]
    [Tooltip("Высота для препятствий (кубов).")]
    public float obstacleSpawnY = 0.5f; //
    
    [Tooltip("Высота для робота на уровне гусениц.")]
    public float robotSpawnY = 0.05f; //[cite: 1]
    
    [Tooltip("Высота для мяча.")]
    public float targetSpawnY = 0.1f; //[cite: 1]

    private List<GameObject> spawnedObstacles = new List<GameObject>();
    
    // Внутренние динамические границы арены
    private float minX, maxX;
    private float minZ, maxZ;

    // Этот метод вызывается в начале каждого эпизода обучения[cite: 1]
    public void ResetArena()
{
    CalculateArenaBounds();

    // 1. Удаляем старые препятствия
    foreach (var obstacle in spawnedObstacles)
    {
        Destroy(obstacle);
    }
    spawnedObstacles.Clear();

    // 2. Ставим робота (препятствий еще нет)
    robotTransform.localPosition = GetRandomLocalPosition(robotSpawnY);

    // 3. Генерируем препятствия
    for (int i = 0; i < numberOfObstacles; i++)
    {
        Vector3 spawnPos = GetValidObstaclePosition(obstacleSpawnY);
        GameObject newObstacle = Instantiate(obstaclePrefab, transform);
        newObstacle.transform.localPosition = spawnPos;
        newObstacle.transform.localRotation = Quaternion.identity;
        spawnedObstacles.Add(newObstacle);
    }

    // 4. Ставим мячик (уже зная позиции препятствий)
    MoveTargetSafely();
}

    private void MoveTargetSafely()
{
    int maxAttempts = 50;
    for (int attempt = 0; attempt < maxAttempts; attempt++)
    {
        Vector3 pos = GetRandomLocalPosition(targetSpawnY);
        
        // Проверка дистанции до робота
        if (Vector3.Distance(pos, robotTransform.localPosition) < safeRadius) continue;

        // Проверка дистанции до всех препятствий
        bool tooCloseToObstacle = false;
        foreach (var obs in spawnedObstacles)
        {
            if (Vector3.Distance(pos, obs.transform.localPosition) < safeRadius)
            {
                tooCloseToObstacle = true;
                break;
            }
        }

        if (!tooCloseToObstacle)
        {
            targetTransform.localPosition = pos;
            return;
        }
    }
    // Если место не найдено за 50 попыток, ставим в центр как запасной вариант
    targetTransform.localPosition = new Vector3(0, targetSpawnY, 0);
}
    private Vector3 GetValidObstaclePosition(float localY)
{
    int maxAttempts = 50;
    for (int attempt = 0; attempt < maxAttempts; attempt++)
    {
        Vector3 pos = GetRandomLocalPosition(localY);
        // Проверяем, чтобы препятствие не спавнилось на роботе
        if (Vector3.Distance(pos, robotTransform.localPosition) > safeRadius)
        {
            return pos;
        }
    }
    return GetRandomLocalPosition(localY);
}

    /// <summary>
    /// Динамически вычисляет локальные границы плоскости пола
    /// </summary>
    private void CalculateArenaBounds()
    {
        if (arenaFloor != null)
        {
            // Получаем локальные размеры меша плоскости
            // Bounds хранит размеры в глобальных координатах, поэтому переводим их в локальные относительно TrainingArea
            Bounds localBounds = arenaFloor.localBounds;

            // Вычисляем масштаб плоскости относительно родителя
            Vector3 floorScale = arenaFloor.transform.localScale;

            // Базовый размер Plane в Unity равен 10x10 единиц, умножаем его на локальный масштаб
            float halfWidthX = (localBounds.size.x * floorScale.x) / 2f;
            float halfLengthZ = (localBounds.size.z * floorScale.z) / 2f;

            // Локальное смещение самого пола относительно центра TrainingArea (если Plane сдвинут)
            Vector3 floorLocalPos = arenaFloor.transform.localPosition;

            // Задаем строгие диапазоны с учетом безопасного отступа от краев
            minX = floorLocalPos.x - halfWidthX + edgePadding;
            maxX = floorLocalPos.x + halfWidthX - edgePadding;
            
            minZ = floorLocalPos.z - halfLengthZ + edgePadding;
            maxZ = floorLocalPos.z + halfLengthZ - edgePadding;
        }
        else
        {
            // Резервный дефолтный вариант, если забыли привязать Plane в инспекторе
            Debug.LogWarning("Arena Floor (Plane) не назначен в ArenaController! Использованы дефолтные размеры 10x10.");
            minX = -4.5f; maxX = 4.5f;
            minZ = -4.5f; maxZ = 4.5f;
        }
    }

    private void MoveObjectRandomly(Transform obj, float localY) //[cite: 1]
    {
        obj.localPosition = GetRandomLocalPosition(localY); //[cite: 1]
    }

    private Vector3 GetRandomLocalPosition(float localY)
    {
        // Генерируем случайную точку строго в вычисленных локальных границах нашего Plane
        float randomX = Random.Range(minX, maxX);
        float randomZ = Random.Range(minZ, maxZ);
        
        // Дополнительный Clamp для 100% защиты от выхода за рамки
        randomX = Mathf.Clamp(randomX, minX, maxX);
        randomZ = Mathf.Clamp(randomZ, minZ, maxZ);

        return new Vector3(randomX, localY, randomZ);
    }

    private Vector3 GetValidSpawnPosition(float localY) //[cite: 1]
    {
        int maxAttempts = 50; //[cite: 1]
        Vector3 potentialLocalPosition = Vector3.zero; //[cite: 1]

        for (int attempt = 0; attempt < maxAttempts; attempt++) //[cite: 1]
        {
            // Работаем строго в локальном пространстве арены[cite: 1]
            potentialLocalPosition = GetRandomLocalPosition(localY);

            // Проверяем расстояния в локальных координатах[cite: 1]
            float distToRobot = Vector3.Distance(potentialLocalPosition, robotTransform.localPosition); //[cite: 1]
            float distToTarget = Vector3.Distance(potentialLocalPosition, targetTransform.localPosition); //[cite: 1]

            // Если точка достаточно далеко от робота и цели[cite: 1]
            if (distToRobot > safeRadius && distToTarget > safeRadius) //[cite: 1]
            {
                bool tooCloseToOtherObstacle = false; //[cite: 1]
                foreach (var obs in spawnedObstacles) //[cite: 1]
                {
                    if (Vector3.Distance(potentialLocalPosition, obs.transform.localPosition) < safeRadius) //[cite: 1]
                    {
                        tooCloseToOtherObstacle = true; //[cite: 1]
                        break; //[cite: 1]
                    }
                }

                if (!tooCloseToOtherObstacle) //[cite: 1]
                {
                    return potentialLocalPosition; // Точка найдена![cite: 1]
                }
            }
        }

        return potentialLocalPosition; // Дефолтный спавн при неудаче[cite: 1]
    }
}