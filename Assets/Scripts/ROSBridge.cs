using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSBridge : MonoBehaviour
{
    private ROSConnection ros;

    [Header("=== Ссылки на скрипты Unity ===")]
    public VirtualSensors virtualSensors;
    public TrackController trackController;

    [Header("=== Топики ROS ===")]
    public string cmdVelTopic = "/cmd_vel";
    public string sensorDataTopic = "/sensor/data";

    [Header("=== Настройки ===")]
    public float publishRateHz = 15f; 
    
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // 1. Подписываемся на команды моторов от твоего Python FSM
        ros.Subscribe<TwistMsg>(cmdVelTopic, OnCmdVelReceived);

        // 2. Регистрируем публикатор сенсоров
        ros.RegisterPublisher<QuaternionMsg>(sensorDataTopic);
    }

    void Update()
    {
        // Публикуем данные с датчиков с заданной частотой
        if (Time.time - lastPublishTime > (1f / publishRateHz))
        {
            PublishSensorData();
            lastPublishTime = Time.time;
        }
    }

    private void PublishSensorData()
    {
        if (virtualSensors == null) return;

        QuaternionMsg msg = new QuaternionMsg();
        msg.x = 0f; // Не используется в твоем Python скрипте
        msg.y = virtualSensors.leftIRObstacle;   // Левый
        msg.z = virtualSensors.rightIRObstacle;  // Правый
        msg.w = virtualSensors.centerIRObstacle; // Центральный

        ros.Publish(sensorDataTopic, msg);
    }

    private void OnCmdVelReceived(TwistMsg msg)
    {
        if (trackController == null) return;

        float linearX = (float)msg.linear.x;
        float angularZ = (float)msg.angular.z;

        // Передаем сырые команды в TrackController. 
        // Знак angularZ инвертирован, чтобы соответствовать физике (как было у вас).
        trackController.SetInputs(linearX, -angularZ, 0f);
    }
}
