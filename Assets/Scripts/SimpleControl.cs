using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleControl : MonoBehaviour
{
    private ROSBridge bridge;

    void Start()
    {
        bridge = GetComponent<ROSBridge>();
    }

    void Update()
    {
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        // --- ДВИЖЕНИЕ (WASD) ---
        float move = 0;
        float turn = 0;
        // wasd by default !!!!!!!!!!!!!!!!!!!!!!!!
        if (keyboard.aKey.isPressed) move = 1f;
        if (keyboard.dKey.isPressed) move = -1f;
        if (keyboard.sKey.isPressed) turn = -1f;
        if (keyboard.wKey.isPressed) turn = 1f;
        bridge.PublishCommand(move, turn);

        // --- КЛЕШНЯ: ОТКРЫТЬ/ЗАКРЫТЬ (Q/E) ---
        if (keyboard.qKey.wasPressedThisFrame) bridge.PublishGripperCmd(1); // Открыть
        if (keyboard.eKey.wasPressedThisFrame) bridge.PublishGripperCmd(2); // Закрыть

        // --- РУКА: ВВЕРХ/ВНИЗ (R/F) ---
        if (keyboard.rKey.isPressed) 
        {
            bridge.PublishArmCmd(1); // 1 - Команда вверх
        }
        else if (keyboard.fKey.isPressed) 
        {
            bridge.PublishArmCmd(2); // 2 - Команда вниз
        }
        else if (keyboard.rKey.wasReleasedThisFrame || keyboard.fKey.wasReleasedThisFrame)
        {
            bridge.PublishArmCmd(0); // 0 - Остановить мотор
        }
    }
}
