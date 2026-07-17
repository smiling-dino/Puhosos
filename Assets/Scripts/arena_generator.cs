using System.Collections.Generic;
using UnityEngine;

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
    public Transform cameraTransform;
    public float targetVisibleMinDistance = 1.55f;
    public float targetVisibleMaxDistance = 1.85f;
    public float targetVisibleHorizontalFov = 40f;
    public float targetVisibleAnglePadding = 2f;

    private const int MaxSpawnAttempts = 150;

    private readonly List<GameObject> spawnedObstacles = new List<GameObject>();
    private float minX;
    private float maxX;
    private float minZ;
    private float maxZ;

    public IReadOnlyList<GameObject> SpawnedObstacles => spawnedObstacles;
    public int LastRejectedObstacleSpawns { get; private set; }
    public int LastRejectedTargetSpawns { get; private set; }
    public int LastRejectedTargetVisibilitySpawns { get; private set; }
    public int LastObstacleFallbackSpawns { get; private set; }
    public int LastTargetFallbackSpawns { get; private set; }
    public int LastVisibleTargetFallbackSpawns { get; private set; }

    public void ResetArena()
    {
        CalculateArenaBounds();
        ResetSpawnDiagnostics();
        ClearSpawnedObstacles();

        if (robotTransform != null)
        {
            robotTransform.localPosition = GetValidRobotPosition(robotSpawnY);
            robotTransform.localRotation = Quaternion.identity;
        }

        for (int i = 0; i < numberOfObstacles; i++)
        {
            Vector3 spawnPos = GetValidObstaclePosition(obstacleSpawnY);
            GameObject newObstacle = Instantiate(obstaclePrefab, transform);
            newObstacle.transform.localPosition = spawnPos;
            newObstacle.transform.localRotation = Quaternion.identity;
            spawnedObstacles.Add(newObstacle);
        }

        MoveTargetSafely();
        ResetTargetPhysics();
    }

    public void ResetTargetPhysics()
    {
        if (targetTransform == null)
        {
            return;
        }

        targetTransform.localRotation = Quaternion.identity;

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

    private void ClearSpawnedObstacles()
    {
        foreach (GameObject obstacle in spawnedObstacles)
        {
            Destroy(obstacle);
        }

        spawnedObstacles.Clear();
    }

    private void ResetSpawnDiagnostics()
    {
        LastRejectedObstacleSpawns = 0;
        LastRejectedTargetSpawns = 0;
        LastRejectedTargetVisibilitySpawns = 0;
        LastObstacleFallbackSpawns = 0;
        LastTargetFallbackSpawns = 0;
        LastVisibleTargetFallbackSpawns = 0;
    }

    private Vector3 GetValidRobotPosition(float localY)
    {
        float robotRadius = GetHorizontalRadius(robotTransform);
        return GetRandomLocalPosition(localY, robotRadius);
    }

    private Vector3 GetValidObstaclePosition(float localY)
    {
        float obstacleRadius = GetHorizontalRadius(obstaclePrefab != null ? obstaclePrefab.transform : null);
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

        if (spawnTargetInsideInitialCameraView && TryMoveTargetIntoInitialCameraView(targetRadius))
        {
            return;
        }

        if (spawnTargetInsideInitialCameraView)
        {
            LastVisibleTargetFallbackSpawns++;
        }

        Vector3 bestPosition = GetRandomLocalPosition(targetSpawnY, targetRadius);
        float bestClearance = float.NegativeInfinity;

        for (int attempt = 0; attempt < MaxSpawnAttempts; attempt++)
        {
            Vector3 candidate = GetRandomLocalPosition(targetSpawnY, targetRadius);
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
            candidate.y = targetSpawnY;

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
        float otherRadius = GetHorizontalRadius(other);
        float requiredDistance = Mathf.Max(safeRadius, candidateRadius + otherRadius + collisionClearance);
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

        return new Vector3(Random.Range(xMin, xMax), localY, Random.Range(zMin, zMax));
    }

    private bool IsInsideSpawnBounds(Vector3 localPosition, float objectRadius)
    {
        GetSpawnRanges(objectRadius, out float xMin, out float xMax, out float zMin, out float zMax);
        return localPosition.x >= xMin
            && localPosition.x <= xMax
            && localPosition.z >= zMin
            && localPosition.z <= zMax;
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
