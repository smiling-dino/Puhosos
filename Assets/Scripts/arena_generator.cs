using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class ArenaController : MonoBehaviour
{
    [Header("Spawn Settings")]
    public GameObject obstaclePrefab;
    public Transform robotTransform;
    public Transform targetTransform;

    [Header("Floor Binding")]
    public MeshRenderer arenaFloor;
    public float edgePadding = 0.5f;

    public int numberOfObstacles = 5;
    public float safeRadius = 1.5f;
    public float collisionClearance = 0.05f;

    [Header("Spawn Heights")]
    public float obstacleSpawnY = 0.5f;
    public float robotSpawnY = 0.05f;
    public float targetSpawnY = 0.1f;

    [Header("Strict Camera Target Spawn")]
    public bool spawnTargetInsideInitialCameraView = true;
    [Range(0f, 1f)] public float defaultInitialVisibleTargetProbability = 0.2f;
    public string initialVisibleTargetProbabilityParameter = "initial_target_visible_probability";
    public Transform cameraTransform;
    public float targetVisibleMinDistance = 1.55f;
    public float targetVisibleMaxDistance = 1.85f;
    public float targetVisibleHorizontalFov = 40f;
    public float targetVisibleAnglePadding = 2f;

    [Header("P1 Domain Randomization")]
    [SerializeField] private float maxFloorTiltDegrees = 1.5f;
    [SerializeField] private Vector2 standardBallScaleRange = new Vector2(0.95f, 1.05f);
    [SerializeField] private Vector2 standardBallMassRange = new Vector2(0.85f, 1.15f);
    [SerializeField] private Vector2 ballFrictionScaleRange = new Vector2(0.7f, 1.3f);
    [SerializeField] private Vector2 ballBouncinessRange = new Vector2(0f, 0.15f);
    [SerializeField] private Vector2 ballLinearDampingRange = new Vector2(0f, 0.05f);
    [SerializeField] private float ballSpawnHeightJitterMeters = 0.005f;
    [SerializeField] private Transform finalTargetTransform;
    [SerializeField] private bool createFinalTargetIfMissing = true;
    [SerializeField] private Vector3 finalTargetDefaultScale = new Vector3(0.2f, 0.2f, 0.2f);
    [SerializeField] private Vector2 finalTargetScaleRange = new Vector2(0.9f, 1.1f);
    [SerializeField] private float finalTargetSpawnY = 0.1f;
    [SerializeField] private float finalTargetClearanceRadius = 0.45f;
    [SerializeField] private Vector2Int randomizedObstacleCountRange = new Vector2Int(3, 10);
    [SerializeField] private Vector2 randomizedObstacleScaleRange = new Vector2(0.8f, 1.2f);

    [Header("P2 Domain Randomization")]
    [SerializeField] private bool useMultipleBallTypes = true;
    [SerializeField] private Vector2 lightBallScaleRange = new Vector2(0.9f, 0.97f);
    [SerializeField] private Vector2 lightBallMassRange = new Vector2(0.7f, 0.9f);
    [SerializeField] private Vector2 heavyBallScaleRange = new Vector2(1.03f, 1.1f);
    [SerializeField] private Vector2 heavyBallMassRange = new Vector2(1.1f, 1.3f);
    [SerializeField] private bool useProceduralFloorTextures = true;
    [SerializeField, Range(0f, 1f)] private float partialOcclusionProbability = 0.3f;
    [SerializeField] private Vector2Int redDistractorCountRange = new Vector2Int(0, 3);
    [SerializeField] private Vector2 redDistractorSizeRange = new Vector2(0.08f, 0.22f);

    private const int MaxSpawnAttempts = 150;

    private readonly List<GameObject> spawnedObstacles = new List<GameObject>();
    private readonly List<GameObject> spawnedOccluders = new List<GameObject>();
    private float minX;
    private float maxX;
    private float minZ;
    private float maxZ;
    private bool p1RandomizationEnabled;
    private bool p2RandomizationEnabled;
    private bool environmentDefaultsCaptured;
    private Quaternion defaultFloorLocalRotation = Quaternion.identity;
    private float defaultFloorSurfaceLocalY;
    private Vector3 defaultBallScale = Vector3.one;
    private float defaultBallMass = 1f;
    private float defaultBallLinearDamping;
    private float defaultBallAngularDamping = 0.05f;
    private PhysicsMaterial defaultBallPhysicsMaterial;
    private PhysicsMaterial runtimeBallPhysicsMaterial;
    private Vector3 capturedFinalTargetScale;
    private Vector3 defaultFinalTargetLocalPosition;
    private Quaternion defaultFinalTargetLocalRotation = Quaternion.identity;
    private MaterialPropertyBlock floorPropertyBlock;
    private MaterialPropertyBlock redPropertyBlock;
    private static Texture2D[] proceduralFloorTextures;
    private float episodeTargetSpawnY;

    public IReadOnlyList<GameObject> SpawnedObstacles => spawnedObstacles;
    public Transform FinalTargetTransform => finalTargetTransform;
    public int LastBallTypeIndex { get; private set; }
    public int LastRejectedObstacleSpawns { get; private set; }
    public int LastRejectedTargetSpawns { get; private set; }
    public int LastRejectedTargetVisibilitySpawns { get; private set; }
    public int LastObstacleFallbackSpawns { get; private set; }
    public int LastTargetFallbackSpawns { get; private set; }
    public int LastVisibleTargetFallbackSpawns { get; private set; }
    public bool LastTargetRequestedVisible { get; private set; }

    public void ConfigureEpisodeRandomization(bool enableP1, bool enableP2)
    {
        p1RandomizationEnabled = enableP1;
        p2RandomizationEnabled = enableP2;
    }

    public void ResetArena()
    {
        CaptureEnvironmentDefaults();
        EnsureFinalTarget();
        ResetSpawnDiagnostics();
        ClearSpawnedObstacles();
        ClearSpawnedOccluders();
        ApplyFloorRandomization();
        ApplyBallRandomization();
        ApplyFinalTargetAppearance();
        CalculateArenaBounds();

        if (robotTransform != null)
        {
            robotTransform.localPosition = GetValidRobotPosition(robotSpawnY);
            robotTransform.localRotation = Quaternion.identity;
        }

        int obstacleCount = p1RandomizationEnabled
            ? RandomInclusive(randomizedObstacleCountRange)
            : numberOfObstacles;

        for (int i = 0; i < obstacleCount; i++)
        {
            GameObject newObstacle = Instantiate(obstaclePrefab, transform);
            if (p1RandomizationEnabled)
            {
                float scaleX = RandomInRange(randomizedObstacleScaleRange);
                float scaleZ = RandomInRange(randomizedObstacleScaleRange);
                newObstacle.transform.localScale = Vector3.Scale(
                    newObstacle.transform.localScale,
                    new Vector3(scaleX, 1f, scaleZ)
                );
                newObstacle.transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            }
            else
            {
                newObstacle.transform.localRotation = Quaternion.identity;
            }

            Vector3 spawnPos = GetValidObstaclePosition(obstacleSpawnY, newObstacle.transform);
            newObstacle.transform.localPosition = spawnPos;
            spawnedObstacles.Add(newObstacle);
        }

        MoveTargetSafely();
        MoveFinalTargetSafely();
        SpawnRedDistractors();
        SpawnPartialOccluders();
        ApplyFloorTextureRandomization();
        ResetTargetPhysics();
        Physics.SyncTransforms();
    }

    public void ResetTargetPhysics()
    {
        if (targetTransform == null)
        {
            return;
        }

        targetTransform.localRotation = (p1RandomizationEnabled || p2RandomizationEnabled)
            ? Random.rotation
            : Quaternion.identity;

        Collider targetCollider = targetTransform.GetComponent<Collider>();
        if (targetCollider != null)
        {
            targetCollider.enabled = true;
        }

        Rigidbody targetRigidbody = targetTransform.GetComponent<Rigidbody>();
        if (targetRigidbody == null)
        {
            return;
        }

        targetRigidbody.linearVelocity = Vector3.zero;
        targetRigidbody.angularVelocity = Vector3.zero;
        targetRigidbody.isKinematic = true;
        targetRigidbody.position = targetTransform.position;
        targetRigidbody.rotation = targetTransform.rotation;
        targetRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        targetRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        Physics.SyncTransforms();
        targetRigidbody.isKinematic = false;
        targetRigidbody.linearVelocity = Vector3.zero;
        targetRigidbody.angularVelocity = Vector3.zero;
        targetRigidbody.WakeUp();
    }

    private void CaptureEnvironmentDefaults()
    {
        if (environmentDefaultsCaptured)
        {
            return;
        }

        episodeTargetSpawnY = targetSpawnY;

        if (arenaFloor != null)
        {
            defaultFloorLocalRotation = arenaFloor.transform.localRotation;
            Vector3 floorReferenceWorld = arenaFloor.transform.TransformPoint(arenaFloor.localBounds.center);
            defaultFloorSurfaceLocalY = transform.InverseTransformPoint(floorReferenceWorld).y;
        }

        if (targetTransform != null)
        {
            defaultBallScale = targetTransform.localScale;

            Rigidbody targetRigidbody = targetTransform.GetComponent<Rigidbody>();
            if (targetRigidbody != null)
            {
                defaultBallMass = targetRigidbody.mass;
                defaultBallLinearDamping = targetRigidbody.linearDamping;
                defaultBallAngularDamping = targetRigidbody.angularDamping;
            }

            Collider targetCollider = targetTransform.GetComponent<Collider>();
            if (targetCollider != null)
            {
                defaultBallPhysicsMaterial = targetCollider.sharedMaterial;
            }
        }

        if (finalTargetTransform != null)
        {
            capturedFinalTargetScale = finalTargetTransform.localScale;
            defaultFinalTargetLocalPosition = finalTargetTransform.localPosition;
            defaultFinalTargetLocalRotation = finalTargetTransform.localRotation;
        }

        environmentDefaultsCaptured = true;
    }

    private void EnsureFinalTarget()
    {
        if (finalTargetTransform != null || !createFinalTargetIfMissing)
        {
            return;
        }

        GameObject finalTarget = GameObject.CreatePrimitive(PrimitiveType.Cube);
        finalTarget.name = "FinalTargetCube";
        finalTarget.transform.SetParent(transform, false);
        finalTarget.transform.localScale = finalTargetDefaultScale;
        finalTarget.transform.localPosition = new Vector3(2f, finalTargetSpawnY, 2f);

        BoxCollider targetCollider = finalTarget.GetComponent<BoxCollider>();
        if (targetCollider != null)
        {
            // Цель задаёт точку остановки, а не дополнительную стену.
            targetCollider.isTrigger = true;
        }

        finalTargetTransform = finalTarget.transform;
        capturedFinalTargetScale = finalTargetTransform.localScale;
        defaultFinalTargetLocalPosition = finalTargetTransform.localPosition;
        defaultFinalTargetLocalRotation = finalTargetTransform.localRotation;
    }

    private void ApplyFloorRandomization()
    {
        if (arenaFloor == null)
        {
            return;
        }

        if (!p1RandomizationEnabled)
        {
            arenaFloor.transform.localRotation = defaultFloorLocalRotation;
            return;
        }

        float pitch = Random.Range(-Mathf.Abs(maxFloorTiltDegrees), Mathf.Abs(maxFloorTiltDegrees));
        float roll = Random.Range(-Mathf.Abs(maxFloorTiltDegrees), Mathf.Abs(maxFloorTiltDegrees));
        arenaFloor.transform.localRotation = defaultFloorLocalRotation * Quaternion.Euler(pitch, 0f, roll);
    }

    private void ApplyBallRandomization()
    {
        if (targetTransform == null)
        {
            return;
        }

        bool randomizeBall = p1RandomizationEnabled || p2RandomizationEnabled;
        episodeTargetSpawnY = targetSpawnY;
        LastBallTypeIndex = 0;

        Vector2 scaleRange = standardBallScaleRange;
        Vector2 massRange = standardBallMassRange;

        if (p2RandomizationEnabled && useMultipleBallTypes)
        {
            LastBallTypeIndex = Random.Range(0, 3);
            if (LastBallTypeIndex == 1)
            {
                scaleRange = lightBallScaleRange;
                massRange = lightBallMassRange;
            }
            else if (LastBallTypeIndex == 2)
            {
                scaleRange = heavyBallScaleRange;
                massRange = heavyBallMassRange;
            }
        }

        float scaleMultiplier = randomizeBall ? RandomInRange(scaleRange) : 1f;
        targetTransform.localScale = defaultBallScale * scaleMultiplier;
        if (randomizeBall)
        {
            episodeTargetSpawnY += Random.Range(
                -Mathf.Abs(ballSpawnHeightJitterMeters),
                Mathf.Abs(ballSpawnHeightJitterMeters)
            );
        }

        Rigidbody targetRigidbody = targetTransform.GetComponent<Rigidbody>();
        if (targetRigidbody != null)
        {
            targetRigidbody.mass = randomizeBall
                ? defaultBallMass * RandomInRange(massRange)
                : defaultBallMass;
            targetRigidbody.linearDamping = randomizeBall
                ? RandomInRange(ballLinearDampingRange)
                : defaultBallLinearDamping;
            targetRigidbody.angularDamping = defaultBallAngularDamping;
        }

        Collider targetCollider = targetTransform.GetComponent<Collider>();
        if (targetCollider == null)
        {
            return;
        }

        if (!randomizeBall)
        {
            targetCollider.sharedMaterial = defaultBallPhysicsMaterial;
            return;
        }

        if (runtimeBallPhysicsMaterial == null)
        {
            runtimeBallPhysicsMaterial = new PhysicsMaterial("Runtime Ball Domain Randomization");
            runtimeBallPhysicsMaterial.hideFlags = HideFlags.DontSave;
        }

        float baseDynamicFriction = defaultBallPhysicsMaterial != null
            ? defaultBallPhysicsMaterial.dynamicFriction
            : 0.6f;
        float baseStaticFriction = defaultBallPhysicsMaterial != null
            ? defaultBallPhysicsMaterial.staticFriction
            : 0.6f;
        float frictionMultiplier = RandomInRange(ballFrictionScaleRange);
        runtimeBallPhysicsMaterial.dynamicFriction = Mathf.Clamp01(baseDynamicFriction * frictionMultiplier);
        runtimeBallPhysicsMaterial.staticFriction = Mathf.Clamp01(baseStaticFriction * frictionMultiplier);
        runtimeBallPhysicsMaterial.bounciness = Mathf.Clamp01(RandomInRange(ballBouncinessRange));
        targetCollider.sharedMaterial = runtimeBallPhysicsMaterial;
    }

    private void ApplyFinalTargetAppearance()
    {
        if (finalTargetTransform == null)
        {
            return;
        }

        float scaleMultiplier = p1RandomizationEnabled ? RandomInRange(finalTargetScaleRange) : 1f;
        finalTargetTransform.localScale = capturedFinalTargetScale * scaleMultiplier;

        Renderer targetRenderer = finalTargetTransform.GetComponentInChildren<Renderer>();
        if (targetRenderer == null)
        {
            return;
        }

        if (!p1RandomizationEnabled && !p2RandomizationEnabled)
        {
            targetRenderer.SetPropertyBlock(null);
            return;
        }

        if (redPropertyBlock == null)
        {
            redPropertyBlock = new MaterialPropertyBlock();
        }

        float hue = Mathf.Repeat(Random.Range(-10f, 10f) / 360f, 1f);
        Color red = Color.HSVToRGB(hue, Random.Range(0.65f, 1f), Random.Range(0.45f, 1f));
        targetRenderer.GetPropertyBlock(redPropertyBlock);
        redPropertyBlock.SetColor("_BaseColor", red);
        redPropertyBlock.SetColor("_Color", red);
        targetRenderer.SetPropertyBlock(redPropertyBlock);
    }

    private void OnDestroy()
    {
        if (runtimeBallPhysicsMaterial != null)
        {
            Destroy(runtimeBallPhysicsMaterial);
        }
    }

    private void ClearSpawnedObstacles()
    {
        foreach (GameObject obstacle in spawnedObstacles)
        {
            if (obstacle != null)
            {
                obstacle.SetActive(false);
                Destroy(obstacle);
            }
        }

        spawnedObstacles.Clear();
    }

    private void ClearSpawnedOccluders()
    {
        foreach (GameObject occluder in spawnedOccluders)
        {
            if (occluder != null)
            {
                occluder.SetActive(false);
                Destroy(occluder);
            }
        }

        spawnedOccluders.Clear();
    }

    private void ResetSpawnDiagnostics()
    {
        LastRejectedObstacleSpawns = 0;
        LastRejectedTargetSpawns = 0;
        LastRejectedTargetVisibilitySpawns = 0;
        LastObstacleFallbackSpawns = 0;
        LastTargetFallbackSpawns = 0;
        LastVisibleTargetFallbackSpawns = 0;
        LastTargetRequestedVisible = false;
    }

    private Vector3 GetValidRobotPosition(float localY)
    {
        float robotRadius = GetHorizontalRadius(robotTransform);
        return GetRandomLocalPosition(localY, robotRadius);
    }

    private Vector3 GetValidObstaclePosition(float localY, Transform obstacleTransform)
    {
        float obstacleRadius = GetHorizontalRadius(obstacleTransform);
        Vector3 bestPosition = GetRandomLocalPosition(localY, obstacleRadius);
        float bestClearance = float.NegativeInfinity;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            Vector3 candidate = GetRandomLocalPosition(localY, obstacleRadius);
            float clearance = GetMinimumClearance(candidate, obstacleRadius, includeRobot: true, includeObstacles: true);

            if (clearance >= 0f)
            {
                return candidate;
            }

            LastRejectedObstacleSpawns++;
            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPosition = candidate;
            }
        }

        LastObstacleFallbackSpawns++;
        return bestPosition;
    }

    private void MoveTargetSafely()
    {
        if (targetTransform == null)
        {
            return;
        }

        float targetRadius = GetHorizontalRadius(targetTransform);

        bool requestVisibleSpawn = ShouldRequestVisibleTargetSpawn();
        LastTargetRequestedVisible = requestVisibleSpawn;

        if (requestVisibleSpawn && TryMoveTargetIntoInitialCameraView(targetRadius))
        {
            return;
        }

        if (requestVisibleSpawn)
        {
            LastVisibleTargetFallbackSpawns++;
        }

        Vector3 bestPosition = GetRandomLocalPosition(episodeTargetSpawnY, targetRadius);
        float bestClearance = float.NegativeInfinity;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            Vector3 candidate = GetRandomLocalPosition(episodeTargetSpawnY, targetRadius);
            float clearance = GetMinimumClearance(candidate, targetRadius, includeRobot: true, includeObstacles: true);

            if (clearance >= 0f)
            {
                targetTransform.localPosition = candidate;
                return;
            }

            LastRejectedTargetSpawns++;
            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPosition = candidate;
            }
        }

        LastTargetFallbackSpawns++;
        targetTransform.localPosition = bestPosition;
    }

    private void MoveFinalTargetSafely()
    {
        if (finalTargetTransform == null)
        {
            return;
        }

        if (!p1RandomizationEnabled)
        {
            finalTargetTransform.localPosition = defaultFinalTargetLocalPosition;
            finalTargetTransform.localRotation = defaultFinalTargetLocalRotation;
            return;
        }

        float targetRadius = GetVisualRadius(finalTargetTransform);
        float spawnRadius = targetRadius + Mathf.Max(0f, finalTargetClearanceRadius);
        Vector3 bestPosition = GetRandomLocalPosition(finalTargetSpawnY, spawnRadius);
        float bestClearance = float.NegativeInfinity;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            Vector3 candidate = GetRandomLocalPosition(finalTargetSpawnY, spawnRadius);
            float clearance = GetFinalTargetClearance(candidate, targetRadius);
            if (clearance >= 0f && HasNavigableRoute(candidate))
            {
                finalTargetTransform.localPosition = candidate;
                finalTargetTransform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
                return;
            }

            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPosition = candidate;
            }
        }

        finalTargetTransform.localPosition = bestPosition;
        finalTargetTransform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }

    private float GetFinalTargetClearance(Vector3 candidate, float candidateRadius)
    {
        float minimumClearance = float.PositiveInfinity;
        float extraClearance = Mathf.Max(0f, finalTargetClearanceRadius);

        if (robotTransform != null)
        {
            minimumClearance = Mathf.Min(
                minimumClearance,
                GetClearance(candidate, candidateRadius, robotTransform, extraClearance)
            );
        }

        if (targetTransform != null)
        {
            minimumClearance = Mathf.Min(
                minimumClearance,
                GetClearance(candidate, candidateRadius, targetTransform, extraClearance)
            );
        }

        foreach (GameObject obstacle in spawnedObstacles)
        {
            if (obstacle == null)
            {
                continue;
            }

            minimumClearance = Mathf.Min(
                minimumClearance,
                GetClearance(candidate, candidateRadius, obstacle.transform, extraClearance)
            );
        }

        return minimumClearance;
    }

    private bool HasNavigableRoute(Vector3 goal)
    {
        if (robotTransform == null)
        {
            return true;
        }

        const float cellSize = 0.25f;
        float agentRadius = GetHorizontalRadius(robotTransform) + collisionClearance;
        float gridMinX = minX + agentRadius;
        float gridMaxX = maxX - agentRadius;
        float gridMinZ = minZ + agentRadius;
        float gridMaxZ = maxZ - agentRadius;

        int width = Mathf.Max(1, Mathf.FloorToInt((gridMaxX - gridMinX) / cellSize) + 1);
        int height = Mathf.Max(1, Mathf.FloorToInt((gridMaxZ - gridMinZ) / cellSize) + 1);
        Vector2Int startCell = ToGridCell(robotTransform.localPosition, gridMinX, gridMinZ, cellSize, width, height);
        Vector2Int goalCell = ToGridCell(goal, gridMinX, gridMinZ, cellSize, width, height);

        List<Vector2> obstacleCenters = new List<Vector2>(spawnedObstacles.Count);
        List<float> obstacleRadii = new List<float>(spawnedObstacles.Count);
        foreach (GameObject obstacle in spawnedObstacles)
        {
            if (obstacle == null)
            {
                continue;
            }

            obstacleCenters.Add(ToXZ(obstacle.transform.localPosition));
            obstacleRadii.Add(GetHorizontalRadius(obstacle.transform));
        }

        bool[,] visited = new bool[width, height];
        Queue<Vector2Int> open = new Queue<Vector2Int>();
        open.Enqueue(startCell);
        visited[startCell.x, startCell.y] = true;

        Vector2Int[] directions =
        {
            new Vector2Int(1, 0),
            new Vector2Int(-1, 0),
            new Vector2Int(0, 1),
            new Vector2Int(0, -1)
        };

        while (open.Count > 0)
        {
            Vector2Int current = open.Dequeue();
            if (current == goalCell)
            {
                return true;
            }

            foreach (Vector2Int direction in directions)
            {
                Vector2Int next = current + direction;
                if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height)
                {
                    continue;
                }

                if (visited[next.x, next.y])
                {
                    continue;
                }

                Vector3 point = new Vector3(
                    gridMinX + next.x * cellSize,
                    0f,
                    gridMinZ + next.y * cellSize
                );
                if (IsNavigationCellBlocked(
                    point,
                    agentRadius,
                    obstacleCenters,
                    obstacleRadii
                ))
                {
                    continue;
                }

                visited[next.x, next.y] = true;
                open.Enqueue(next);
            }
        }

        return false;
    }

    private static bool IsNavigationCellBlocked(
        Vector3 point,
        float agentRadius,
        IReadOnlyList<Vector2> obstacleCenters,
        IReadOnlyList<float> obstacleRadii)
    {
        Vector2 point2D = new Vector2(point.x, point.z);
        for (int i = 0; i < obstacleCenters.Count; i++)
        {
            float blockedRadius = obstacleRadii[i] + agentRadius;
            if (Vector2.Distance(point2D, obstacleCenters[i]) < blockedRadius)
            {
                return true;
            }
        }

        return false;
    }

    private static Vector2Int ToGridCell(
        Vector3 localPosition,
        float gridMinX,
        float gridMinZ,
        float cellSize,
        int width,
        int height)
    {
        return new Vector2Int(
            Mathf.Clamp(Mathf.RoundToInt((localPosition.x - gridMinX) / cellSize), 0, width - 1),
            Mathf.Clamp(Mathf.RoundToInt((localPosition.z - gridMinZ) / cellSize), 0, height - 1)
        );
    }

    private void SpawnRedDistractors()
    {
        if (!p2RandomizationEnabled)
        {
            return;
        }

        int distractorCount = RandomInclusive(redDistractorCountRange);
        for (int i = 0; i < distractorCount; i++)
        {
            GameObject distractor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            distractor.name = "RedDistractor_" + i;
            distractor.transform.SetParent(transform, false);

            float size = RandomInRange(redDistractorSizeRange);
            distractor.transform.localScale = new Vector3(
                size * Random.Range(0.8f, 1.2f),
                size * Random.Range(0.8f, 1.2f),
                size * Random.Range(0.8f, 1.2f)
            );
            distractor.transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            distractor.transform.localPosition = GetValidVisualObjectPosition(
                distractor.transform,
                Mathf.Max(0.05f, size * 0.5f)
            );

            BoxCollider collider = distractor.GetComponent<BoxCollider>();
            if (collider != null)
            {
                collider.isTrigger = true;
            }

            Renderer renderer = distractor.GetComponent<Renderer>();
            if (renderer != null)
            {
                if (redPropertyBlock == null)
                {
                    redPropertyBlock = new MaterialPropertyBlock();
                }

                redPropertyBlock.Clear();
                float hue = Mathf.Repeat(Random.Range(-10f, 10f) / 360f, 1f);
                Color red = Color.HSVToRGB(hue, Random.Range(0.7f, 1f), Random.Range(0.45f, 0.95f));
                redPropertyBlock.SetColor("_BaseColor", red);
                redPropertyBlock.SetColor("_Color", red);
                renderer.SetPropertyBlock(redPropertyBlock);
            }

            spawnedOccluders.Add(distractor);
        }
    }

    private Vector3 GetValidVisualObjectPosition(Transform visualTransform, float localY)
    {
        float radius = GetHorizontalRadius(visualTransform);
        Vector3 bestPosition = GetRandomLocalPosition(localY, radius);
        float bestClearance = float.NegativeInfinity;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            Vector3 candidate = GetRandomLocalPosition(localY, radius);
            float clearance = GetMinimumClearance(candidate, radius, includeRobot: true, includeObstacles: true);
            if (targetTransform != null)
            {
                clearance = Mathf.Min(clearance, GetClearance(candidate, radius, targetTransform));
            }
            if (finalTargetTransform != null)
            {
                clearance = Mathf.Min(clearance, GetClearance(candidate, radius, finalTargetTransform));
            }

            if (clearance >= 0f)
            {
                return candidate;
            }

            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPosition = candidate;
            }
        }

        return bestPosition;
    }

    private void SpawnPartialOccluders()
    {
        if (!p2RandomizationEnabled)
        {
            return;
        }

        if (targetTransform != null && Random.value < partialOcclusionProbability)
        {
            CreatePartialOccluder(targetTransform, "BallPartialOccluder");
        }

        if (finalTargetTransform != null && Random.value < partialOcclusionProbability)
        {
            CreatePartialOccluder(finalTargetTransform, "CubePartialOccluder");
        }
    }

    private void CreatePartialOccluder(Transform target, string objectName)
    {
        Transform viewTransform = ResolveCameraTransform();
        if (viewTransform == null || target == null)
        {
            return;
        }

        Renderer targetRenderer = target.GetComponentInChildren<Renderer>();
        Vector3 targetCenter = targetRenderer != null ? targetRenderer.bounds.center : target.position;
        Vector3 cameraToTarget = targetCenter - viewTransform.position;
        if (cameraToTarget.sqrMagnitude < 0.0001f)
        {
            return;
        }

        float targetRadius = GetVisualRadius(target);
        Vector3 direction = cameraToTarget.normalized;
        Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
        if (right.sqrMagnitude < 0.0001f)
        {
            right = viewTransform.right;
        }

        GameObject occluder = GameObject.CreatePrimitive(PrimitiveType.Cube);
        occluder.name = objectName;
        occluder.transform.SetParent(transform, true);
        occluder.transform.position = targetCenter
            - direction * Mathf.Max(0.06f, targetRadius * 2.5f)
            + right * Random.Range(-0.45f, 0.45f) * targetRadius
            + Vector3.up * Random.Range(-0.2f, 0.2f) * targetRadius;
        occluder.transform.rotation = Quaternion.LookRotation(direction, Vector3.up);
        occluder.transform.localScale = new Vector3(
            Mathf.Max(0.01f, targetRadius * Random.Range(0.7f, 1.1f)),
            Mathf.Max(0.02f, targetRadius * Random.Range(1.1f, 1.7f)),
            Mathf.Max(0.008f, targetRadius * 0.35f)
        );

        Renderer occluderRenderer = occluder.GetComponent<Renderer>();
        if (occluderRenderer != null)
        {
            MaterialPropertyBlock block = new MaterialPropertyBlock();
            Color color = Random.ColorHSV(0f, 1f, 0.05f, 0.2f, 0.25f, 0.65f);
            block.SetColor("_BaseColor", color);
            block.SetColor("_Color", color);
            occluderRenderer.SetPropertyBlock(block);
        }

        Collider occluderCollider = occluder.GetComponent<Collider>();
        if (occluderCollider != null)
        {
            Collider targetCollider = target.GetComponentInChildren<Collider>();
            if (targetCollider != null)
            {
                Physics.IgnoreCollision(occluderCollider, targetCollider, true);
            }

            if (robotTransform != null)
            {
                foreach (Collider robotCollider in robotTransform.GetComponentsInChildren<Collider>())
                {
                    Physics.IgnoreCollision(occluderCollider, robotCollider, true);
                }
            }
        }

        spawnedOccluders.Add(occluder);
    }

    private float GetVisualRadius(Transform target)
    {
        Renderer renderer = target.GetComponentInChildren<Renderer>();
        if (renderer == null)
        {
            return Mathf.Max(0.01f, GetHorizontalRadius(target));
        }

        Vector3 extents = renderer.bounds.extents;
        return Mathf.Max(0.01f, Mathf.Min(extents.x, Mathf.Min(extents.y, extents.z)));
    }

    private void ApplyFloorTextureRandomization()
    {
        if (arenaFloor == null)
        {
            return;
        }

        if (!p2RandomizationEnabled || !useProceduralFloorTextures)
        {
            arenaFloor.SetPropertyBlock(null);
            return;
        }

        EnsureProceduralFloorTextures();
        if (floorPropertyBlock == null)
        {
            floorPropertyBlock = new MaterialPropertyBlock();
        }

        arenaFloor.GetPropertyBlock(floorPropertyBlock);
        Texture2D texture = proceduralFloorTextures[Random.Range(0, proceduralFloorTextures.Length)];
        float tiling = Random.Range(2f, 8f);
        Color tint = Random.ColorHSV(0f, 1f, 0.02f, 0.12f, 0.45f, 0.85f);
        floorPropertyBlock.SetTexture("_BaseMap", texture);
        floorPropertyBlock.SetTexture("_MainTex", texture);
        floorPropertyBlock.SetVector("_BaseMap_ST", new Vector4(tiling, tiling, 0f, 0f));
        floorPropertyBlock.SetVector("_MainTex_ST", new Vector4(tiling, tiling, 0f, 0f));
        floorPropertyBlock.SetColor("_BaseColor", tint);
        floorPropertyBlock.SetColor("_Color", tint);
        arenaFloor.SetPropertyBlock(floorPropertyBlock);
    }

    private static void EnsureProceduralFloorTextures()
    {
        if (proceduralFloorTextures != null)
        {
            return;
        }

        proceduralFloorTextures = new Texture2D[3];
        for (int pattern = 0; pattern < proceduralFloorTextures.Length; pattern++)
        {
            Texture2D texture = new Texture2D(16, 16, TextureFormat.RGBA32, false, true);
            texture.name = "RuntimeFloorPattern_" + pattern;
            texture.wrapMode = TextureWrapMode.Repeat;
            texture.filterMode = FilterMode.Bilinear;
            texture.hideFlags = HideFlags.DontSave;

            for (int y = 0; y < texture.height; y++)
            {
                for (int x = 0; x < texture.width; x++)
                {
                    float value;
                    if (pattern == 0)
                    {
                        value = ((x / 4 + y / 4) % 2 == 0) ? 0.7f : 0.35f;
                    }
                    else if (pattern == 1)
                    {
                        value = (x % 5 < 2) ? 0.65f : 0.4f;
                    }
                    else
                    {
                        int hash = (x * 73856093) ^ (y * 19349663);
                        value = 0.35f + (Mathf.Abs(hash) % 100) / 333f;
                    }

                    texture.SetPixel(x, y, new Color(value, value, value, 1f));
                }
            }

            texture.Apply(false, true);
            proceduralFloorTextures[pattern] = texture;
        }
    }

    private bool ShouldRequestVisibleTargetSpawn()
    {
        if (!spawnTargetInsideInitialCameraView)
        {
            return false;
        }

        float visibleProbability = Mathf.Clamp01(defaultInitialVisibleTargetProbability);
        if (Academy.IsInitialized && !string.IsNullOrWhiteSpace(initialVisibleTargetProbabilityParameter))
        {
            visibleProbability = Mathf.Clamp01(
                Academy.Instance.EnvironmentParameters.GetWithDefault(
                    initialVisibleTargetProbabilityParameter,
                    visibleProbability
                )
            );
        }

        return Random.value < visibleProbability;
    }

    private bool TryMoveTargetIntoInitialCameraView(float targetRadius)
    {
        Transform viewTransform = ResolveCameraTransform();
        if (viewTransform == null || robotTransform == null)
        {
            return false;
        }

        float minDistance = Mathf.Min(targetVisibleMinDistance, targetVisibleMaxDistance);
        float maxDistance = Mathf.Max(targetVisibleMinDistance, targetVisibleMaxDistance);
        float halfFov = Mathf.Max(0f, targetVisibleHorizontalFov * 0.5f - Mathf.Max(0f, targetVisibleAnglePadding));

        Vector3 bestPosition = targetTransform.localPosition;
        float bestClearance = float.NegativeInfinity;
        bool hasBestPosition = false;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            robotTransform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

            Vector3 flatForward = Vector3.ProjectOnPlane(viewTransform.forward, Vector3.up);
            if (flatForward.sqrMagnitude < 0.0001f)
            {
                LastRejectedTargetVisibilitySpawns++;
                continue;
            }

            float horizontalAngle = Random.Range(-halfFov, halfFov);
            Vector3 direction = Quaternion.AngleAxis(horizontalAngle, Vector3.up) * flatForward.normalized;
            float distance = Random.Range(minDistance, maxDistance);
            Vector3 worldPosition = viewTransform.position + direction * distance;
            Vector3 candidate = transform.InverseTransformPoint(worldPosition);
            candidate.y = GetFloorAdjustedY(candidate.x, candidate.z, episodeTargetSpawnY);

            if (!IsInsideSpawnBounds(candidate, targetRadius))
            {
                LastRejectedTargetVisibilitySpawns++;
                continue;
            }

            float clearance = GetMinimumClearance(candidate, targetRadius, includeRobot: true, includeObstacles: true);
            if (clearance >= 0f)
            {
                targetTransform.localPosition = candidate;
                return true;
            }

            LastRejectedTargetVisibilitySpawns++;
            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPosition = candidate;
                hasBestPosition = true;
            }
        }

        if (hasBestPosition && bestClearance >= -collisionClearance)
        {
            targetTransform.localPosition = bestPosition;
            return true;
        }

        return false;
    }

    private Transform ResolveCameraTransform()
    {
        if (cameraTransform != null)
        {
            return cameraTransform;
        }

        if (robotTransform == null)
        {
            return null;
        }

        SimulatedYoloCamera yoloCamera = robotTransform.GetComponentInChildren<SimulatedYoloCamera>(true);
        if (yoloCamera == null)
        {
            return null;
        }

        cameraTransform = yoloCamera.transform;
        return cameraTransform;
    }

    private float GetMinimumClearance(Vector3 candidate, float candidateRadius, bool includeRobot, bool includeObstacles)
    {
        float minimumClearance = float.PositiveInfinity;

        if (includeRobot && robotTransform != null)
        {
            minimumClearance = Mathf.Min(minimumClearance, GetClearance(candidate, candidateRadius, robotTransform));
        }

        if (includeObstacles)
        {
            foreach (GameObject obstacle in spawnedObstacles)
            {
                if (obstacle == null)
                {
                    continue;
                }

                minimumClearance = Mathf.Min(minimumClearance, GetClearance(candidate, candidateRadius, obstacle.transform));
            }
        }

        return minimumClearance;
    }

    private float GetClearance(Vector3 candidate, float candidateRadius, Transform other)
    {
        return GetClearance(candidate, candidateRadius, other, 0f);
    }

    private float GetClearance(
        Vector3 candidate,
        float candidateRadius,
        Transform other,
        float extraClearance)
    {
        float otherRadius = other == finalTargetTransform
            ? GetVisualRadius(other)
            : GetHorizontalRadius(other);
        float requiredDistance = Mathf.Max(
            safeRadius,
            candidateRadius + otherRadius + collisionClearance
        ) + Mathf.Max(0f, extraClearance);
        return Vector2.Distance(ToXZ(candidate), ToXZ(other.localPosition)) - requiredDistance;
    }

    private Vector3 GetRandomLocalPosition(float localY, float objectRadius)
    {
        GetSpawnRanges(objectRadius, out float xMin, out float xMax, out float zMin, out float zMax);

        if (xMin > xMax)
        {
            float center = (minX + maxX) * 0.5f;
            xMin = center;
            xMax = center;
        }

        if (zMin > zMax)
        {
            float center = (minZ + maxZ) * 0.5f;
            zMin = center;
            zMax = center;
        }

        float x = Random.Range(xMin, xMax);
        float z = Random.Range(zMin, zMax);
        return new Vector3(x, GetFloorAdjustedY(x, z, localY), z);
    }

    private bool IsInsideSpawnBounds(Vector3 localPosition, float objectRadius)
    {
        GetSpawnRanges(objectRadius, out float xMin, out float xMax, out float zMin, out float zMax);
        return localPosition.x >= xMin
            && localPosition.x <= xMax
            && localPosition.z >= zMin
            && localPosition.z <= zMax;
    }

    private float GetFloorAdjustedY(float localX, float localZ, float configuredY)
    {
        if (arenaFloor == null)
        {
            return configuredY;
        }

        Vector3 floorReference = arenaFloor.transform.TransformPoint(arenaFloor.localBounds.center);
        Plane floorPlane = new Plane(arenaFloor.transform.up, floorReference);
        Vector3 rayOrigin = transform.TransformPoint(
            new Vector3(localX, defaultFloorSurfaceLocalY + 10f, localZ)
        );
        Ray ray = new Ray(rayOrigin, -transform.up);

        if (!floorPlane.Raycast(ray, out float enter))
        {
            return configuredY;
        }

        float currentSurfaceY = transform.InverseTransformPoint(ray.GetPoint(enter)).y;
        return configuredY + currentSurfaceY - defaultFloorSurfaceLocalY;
    }

    private void GetSpawnRanges(float objectRadius, out float xMin, out float xMax, out float zMin, out float zMax)
    {
        float extraMargin = Mathf.Max(0f, objectRadius - edgePadding);
        xMin = minX + extraMargin;
        xMax = maxX - extraMargin;
        zMin = minZ + extraMargin;
        zMax = maxZ - extraMargin;
    }

    private float GetHorizontalRadius(Transform root)
    {
        if (root == null)
        {
            return collisionClearance;
        }

        float radius = 0f;
        Collider[] colliders = root.GetComponentsInChildren<Collider>();

        foreach (Collider collider in colliders)
        {
            if (collider == null || !collider.enabled || collider.isTrigger)
            {
                continue;
            }

            radius = Mathf.Max(radius, GetColliderHorizontalRadius(root, collider));
        }

        return Mathf.Max(radius, collisionClearance);
    }

    private float GetColliderHorizontalRadius(Transform root, Collider collider)
    {
        if (collider is BoxCollider boxCollider)
        {
            return GetLocalBoundsHorizontalRadius(root, boxCollider.transform, new Bounds(boxCollider.center, boxCollider.size));
        }

        if (collider is SphereCollider sphereCollider)
        {
            Vector3 size = Vector3.one * (sphereCollider.radius * 2f);
            return GetLocalBoundsHorizontalRadius(root, sphereCollider.transform, new Bounds(sphereCollider.center, size));
        }

        if (collider is CapsuleCollider capsuleCollider)
        {
            Vector3 size = Vector3.one * (capsuleCollider.radius * 2f);
            size[capsuleCollider.direction] = capsuleCollider.height;
            return GetLocalBoundsHorizontalRadius(root, capsuleCollider.transform, new Bounds(capsuleCollider.center, size));
        }

        if (collider is MeshCollider meshCollider && meshCollider.sharedMesh != null)
        {
            return GetLocalBoundsHorizontalRadius(root, meshCollider.transform, meshCollider.sharedMesh.bounds);
        }

        Bounds bounds = collider.bounds;
        return GetWorldBoundsHorizontalRadius(root, bounds);
    }

    private float GetLocalBoundsHorizontalRadius(Transform root, Transform colliderTransform, Bounds localBounds)
    {
        Vector3 min = localBounds.min;
        Vector3 max = localBounds.max;
        float radius = 0f;

        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(min.x, min.y, min.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(min.x, min.y, max.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(max.x, min.y, min.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(max.x, min.y, max.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(min.x, max.y, min.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(min.x, max.y, max.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(max.x, max.y, min.z))));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, colliderTransform.TransformPoint(new Vector3(max.x, max.y, max.z))));

        return radius;
    }

    private float GetWorldBoundsHorizontalRadius(Transform root, Bounds bounds)
    {
        Vector3 min = bounds.min;
        Vector3 max = bounds.max;
        float radius = 0f;

        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(min.x, min.y, min.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(min.x, min.y, max.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(max.x, min.y, min.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(max.x, min.y, max.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(min.x, max.y, min.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(min.x, max.y, max.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(max.x, max.y, min.z)));
        radius = Mathf.Max(radius, GetHorizontalRadiusAtCorner(root, new Vector3(max.x, max.y, max.z)));

        return radius;
    }

    private float GetHorizontalRadiusAtCorner(Transform root, Vector3 worldPoint)
    {
        Vector3 localOffset;

        if (root.IsChildOf(transform))
        {
            localOffset = transform.InverseTransformPoint(worldPoint) - root.localPosition;
        }
        else
        {
            localOffset = Vector3.Scale(root.InverseTransformPoint(worldPoint), root.lossyScale);
        }

        return Mathf.Sqrt(localOffset.x * localOffset.x + localOffset.z * localOffset.z);
    }

    private Vector2 ToXZ(Vector3 localPosition)
    {
        return new Vector2(localPosition.x, localPosition.z);
    }

    private static float RandomInRange(Vector2 range)
    {
        return Random.Range(Mathf.Min(range.x, range.y), Mathf.Max(range.x, range.y));
    }

    private static int RandomInclusive(Vector2Int range)
    {
        int minimum = Mathf.Min(range.x, range.y);
        int maximum = Mathf.Max(range.x, range.y);
        return Random.Range(minimum, maximum + 1);
    }

    private void CalculateArenaBounds()
    {
        if (arenaFloor != null)
        {
            Bounds localBounds = arenaFloor.localBounds;
            Vector3 floorScale = arenaFloor.transform.localScale;
            Vector3 floorLocalPos = arenaFloor.transform.localPosition;

            float halfWidthX = (localBounds.size.x * floorScale.x) * 0.5f;
            float halfLengthZ = (localBounds.size.z * floorScale.z) * 0.5f;

            minX = floorLocalPos.x - halfWidthX + edgePadding;
            maxX = floorLocalPos.x + halfWidthX - edgePadding;
            minZ = floorLocalPos.z - halfLengthZ + edgePadding;
            maxZ = floorLocalPos.z + halfLengthZ - edgePadding;
        }
        else
        {
            Debug.LogWarning("Arena Floor (Plane) is not assigned in ArenaController. Default 10x10 bounds are used.", this);
            minX = -4.5f;
            maxX = 4.5f;
            minZ = -4.5f;
            maxZ = 4.5f;
        }
    }
}
