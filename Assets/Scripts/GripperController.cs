using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("=== Настройки захвата ===")]
    [Tooltip("Точка между губками клешни, куда будет 'прилипать' мяч")]
    public Transform holdPoint;
    
    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;
    
    // Переменная для запоминания Арены, к которой изначально принадлежал мяч
    private Transform originalParent; 

    [ContextMenu("ТЕСТ: Захватить Мяч")]
    public void TestGrab()
    {
        GameObject ball = GameObject.FindWithTag("TargetBall");
        if (ball != null)
        {
            GrabBall(ball);
        }
        else
        {
            Debug.LogError("Тестовый мяч с тегом 'TargetBall' не найден на сцене!");
        }
    }

    [ContextMenu("ТЕСТ: Отпустить Мяч")]
    public void TestRelease()
    {
        ReleaseBall();
    }

    public void GrabBall(GameObject ball)
    {
        if (heldBall != null) return; 

        heldBall = ball;
        
        // Запоминаем родительский контейнер (Арену), чтобы вернуть мяч туда при отпускании
        originalParent = ball.transform.parent; 
        
        ballRigidbody = heldBall.GetComponent<Rigidbody>();
        ballCollider = heldBall.GetComponent<Collider>();

        if (ballRigidbody != null && ballCollider != null)
        {
            ballRigidbody.isKinematic = true;
            ballCollider.isTrigger = true;
            heldBall.transform.SetParent(holdPoint);
            heldBall.transform.localPosition = Vector3.zero;
        }
    }

    public void ReleaseBall()
    {
        if (heldBall == null) return; 

        if (ballRigidbody != null && ballCollider != null)
        {
            // Возвращаем мяч обратно в его родную Арену
            heldBall.transform.SetParent(originalParent); 
            
            ballCollider.isTrigger = false;
            ballRigidbody.isKinematic = false;
        }

        heldBall = null;
        ballRigidbody = null;
        ballCollider = null;
        originalParent = null; // Очищаем ссылку
    }
    
    public bool IsHoldingBall()
    {
        return heldBall != null;
    }
}