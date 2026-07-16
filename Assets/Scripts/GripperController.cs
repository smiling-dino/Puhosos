using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("=== Настройки захвата ===")]
    [Tooltip("Точка между губками клешни, куда будет 'прилипать' мяч")]
    public Transform holdPoint;
    
    // Ссылки на текущий захваченный мяч и его компоненты
    private GameObject heldBall;
    private Rigidbody ballRigidbody;
    private Collider ballCollider;

    [ContextMenu("ТЕСТ: Захватить Мяч")]
    public void TestGrab()
    {
        // Ищем мяч на сцене по тегу
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


    /// <summary>
    /// Метод захвата мяча (Логическое удержание)
    /// </summary>
    public void GrabBall(GameObject ball)
    {
        if (heldBall != null) return; // Защита: мы уже что-то держим

        heldBall = ball;
        ballRigidbody = heldBall.GetComponent<Rigidbody>();
        ballCollider = heldBall.GetComponent<Collider>();

        if (ballRigidbody != null && ballCollider != null)
        {
            // 1. Отключаем физику (мяч перестает падать и реагировать на силы)
            ballRigidbody.isKinematic = true;
            
            // 2. Отключаем коллизии (чтобы мяч не конфликтовал с коллайдерами клешни)
            ballCollider.enabled = false;
            
            // 3. Делаем мяч дочерним объектом нашей невидимой точки
            heldBall.transform.SetParent(holdPoint);
            
            // 4. Мгновенно центрируем мяч ровно между губками (по координатам HoldPoint)
            heldBall.transform.localPosition = Vector3.zero;
        }
    }

    /// <summary>
    /// Метод разжатия клешни и освобождения мяча
    /// </summary>
    public void ReleaseBall()
    {
        if (heldBall == null) return; // Защита: клешня пуста, отпускать нечего

        if (ballRigidbody != null && ballCollider != null)
        {
            // 1. Отвязываем мяч от клешни (выкидываем его в корень сцены)
            heldBall.transform.SetParent(null);
            
            // 2. Включаем коллайдер обратно
            ballCollider.enabled = true;
            
            // 3. Выключаем кинематику, возвращая мячу гравитацию и физику
            ballRigidbody.isKinematic = false;
        }

        // Очищаем переменные, чтобы клешня снова была готова к работе
        heldBall = null;
        ballRigidbody = null;
        ballCollider = null;
    }
    
    // (Опционально) Метод-проверка, держит ли сейчас клешня мяч
    public bool IsHoldingBall()
    {
        return heldBall != null;
    }
}