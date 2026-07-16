using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("=== Настройки захвата ===")]
    [Tooltip("Точка между губками клешни")]
    public Transform holdPoint;

    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;

    // Родитель мяча до захвата — TrainingArea конкретного агента
    private Transform originalBallParent;

    public void GrabBall(GameObject ball)
    {
        if (heldBall != null || ball == null)
        {
            return;
        }

        if (holdPoint == null)
        {
            Debug.LogError("HoldPoint не назначен в GripperController", this);
            return;
        }

        heldBall = ball;
        ballRigidbody = heldBall.GetComponent<Rigidbody>();
        ballCollider = heldBall.GetComponent<Collider>();

        // Запоминаем арену, которой принадлежит мяч
        originalBallParent = heldBall.transform.parent;

        if (ballRigidbody != null)
        {
            ballRigidbody.linearVelocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
            ballRigidbody.isKinematic = true;
        }

        if (ballCollider != null)
        {
            ballCollider.enabled = false;
        }

        // Прикрепляем мяч к клешне
        heldBall.transform.SetParent(holdPoint, false);
        heldBall.transform.localPosition = Vector3.zero;
        heldBall.transform.localRotation = Quaternion.identity;
    }

    public void ReleaseBall()
    {
        if (heldBall == null)
        {
            return;
        }

        // Возвращаем мяч обратно в его TrainingArea
        Transform targetParent = originalBallParent != null
            ? originalBallParent
            : transform.parent;

        heldBall.transform.SetParent(targetParent, true);

        if (ballCollider != null)
        {
            ballCollider.enabled = true;
        }

        if (ballRigidbody != null)
        {
            ballRigidbody.isKinematic = false;
            ballRigidbody.linearVelocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
        }

        heldBall = null;
        ballRigidbody = null;
        ballCollider = null;
        originalBallParent = null;
    }

    public bool IsHoldingBall()
    {
        return heldBall != null;
    }
}