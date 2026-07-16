using UnityEngine;

public class ManualInput : MonoBehaviour
{
    private ROSBridge rosBridge;

    [Header("Settings")]
    public bool isManualControlEnabled = true;

    void Start()
    {
        rosBridge = GetComponent<ROSBridge>();
    }

    void Update()
    {
        if (!isManualControlEnabled) return;

        // Считываем оси WASD или стрелки
        // Vertical: W (1), S (-1)
        // Horizontal: D (1), A (-1)
        float gas = Input.GetAxis("Vertical");
        float steering = Input.GetAxis("Horizontal");

        // Отправляем команды в мост, если нажата хоть одна клавиша
        // Или постоянно, чтобы Watchdog видел, что мы "живы"
        if (rosBridge != null)
        {
            rosBridge.PublishCommand(gas, steering);
        }

        // Проверка кнопок для захвата (например, E - закрыть, Q - открыть)
        if (Input.GetKeyDown(KeyCode.E)) rosBridge.PublishGripperCmd(2);
        if (Input.GetKeyDown(KeyCode.Q)) rosBridge.PublishGripperCmd(1);
    }
}
