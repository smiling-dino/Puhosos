using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSBridge : MonoBehaviour
{
    private ROSConnection ros;

    [Header("=== Топики ROS ===")]
    public string cmdVelTopic = "/cmd_vel";
    public string cmdGripperTopic = "/cmd_gripper";
    public string cmdCameraPanTopic = "/cmd_camera_pan";

    [Header("=== Лимиты робота ===")]
    public float maxLinearSpeed = 0.5f;   // м/с
    public float maxAngularSpeed = 1.5f;  // рад/с

    [Header("=== Настройки безопасности и сглаживания ===")]
    [Range(0.1f, 1f)]
    [Tooltip("Коэффициент сглаживания: 1 - без сглаживания, 0.1 - максимальная плавность")]
    public float emaAlpha = 0.8f;         
    
    [Tooltip("Время в секундах без команд до экстренной остановки (Watchdog)")]
    public float watchdogTimeout = 0.5f;

    // Внутренние переменные
    private float smoothGas = 0f;
    private float smoothSteer = 0f;
    private float lastCommandTime;
    private bool isEmergencyStopped = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Регистрация топиков
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<Int32Msg>(cmdGripperTopic);
        ros.RegisterPublisher<Float32Msg>(cmdCameraPanTopic);

        lastCommandTime = Time.time;
    }

    void Update()
    {
        // РЕАЛИЗАЦИЯ ЗАДАЧИ ДЛЯ СТУДЕНТОВ: Watchdog / Fail-safe
        if (Time.time - lastCommandTime > watchdogTimeout && !isEmergencyStopped)
        {
            ApplyEmergencyStop();
        }
    }

    /// <summary>
    /// Основной метод управления движением (с фильтром EMA)
    /// </summary>
    public void PublishCmdVel(float gas, float steer)
    {
        // Обновляем таймер Watchdog
        lastCommandTime = Time.time;
        isEmergencyStopped = false;

        // Защита от дрейфа (Hard Stop)
        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steer, 0f))
        {
            smoothGas = 0f;
            smoothSteer = 0f;
        }
        else
        {
            // Формула экспоненциального сглаживания (EMA)
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteer = emaAlpha * steer + (1f - emaAlpha) * smoothSteer;
        }

        TwistMsg twist = new TwistMsg();
        twist.linear.x = smoothGas * maxLinearSpeed;
        // Оставлен минус из изначального кода друга для инверсии оси
        twist.angular.z = -smoothSteer * maxAngularSpeed; 

        ros.Publish(cmdVelTopic, twist);
    }

    /// <summary>
    /// Экстренная остановка при потере связи с агентом
    /// </summary>
    private void ApplyEmergencyStop()
    {
        smoothGas = 0f;
        smoothSteer = 0f;

        TwistMsg stopMsg = new TwistMsg();
        stopMsg.linear.x = 0f;
        stopMsg.angular.z = 0f;

        ros.Publish(cmdVelTopic, stopMsg);
        Debug.LogWarning($"<color=red>[WATCHDOG]</color> Команды от ИИ не поступали более {watchdogTimeout}с! Выполнена экстренная остановка.");
        
        isEmergencyStopped = true; // Блокируем спам сообщениями в консоль
    }

    /// <summary>
    /// Управление клешней
    /// </summary>
    public void PublishGripperCmd(int actionID)
    {
        Int32Msg msg = new Int32Msg { data = actionID };
        ros.Publish(cmdGripperTopic, msg);
    }

    /// <summary>
    /// Управление камерой
    /// </summary>
    public void PublishCameraCmd(float angleDegrees)
    {
        Float32Msg msg = new Float32Msg { data = angleDegrees };
        ros.Publish(cmdCameraPanTopic, msg);
    }

    // ==========================================
    // МЕТОДЫ СОВМЕСТИМОСТИ
    // ==========================================
    public void PublishCommand(float gas, float steer)
    {
        PublishCmdVel(gas, steer);
    }

    public void PublishArmCmd(int actionID)
    {
        PublishGripperCmd(actionID);
    }
}