using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSBridge : MonoBehaviour
{
    private ROSConnection ros;

    [Header("ROS Command Topics")]
    public string cmdVelTopic = "/cmd_vel";
    public string cmdGripperTopic = "/cmd_gripper";
    public string cmdCameraPanTopic = "/cmd_camera_pan";

    [Header("Robot Speed Limits")]
    public float maxLinearSpeed = 0.5f;   // макс. скорость в м/с
    public float maxAngularSpeed = 1.5f;  // макс. скорость поворота в рад/с

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Регистрируем топики отправки команд на робота
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<Int32Msg>(cmdGripperTopic);
        ros.RegisterPublisher<Float32Msg>(cmdCameraPanTopic);
    }

    /// <summary>
    /// Отправка скоростей на моторы (газ и поворот руля)
    /// </summary>
    public void PublishCmdVel(float gas, float steer)
    {
        TwistMsg twist = new TwistMsg();

        // Переводим выход нейросети [-1, 1] в реальные физические лимиты
        twist.linear.x = gas * maxLinearSpeed;
        twist.angular.z = -steer * maxAngularSpeed; 

        ros.Publish(cmdVelTopic, twist);
    }

    /// <summary>
    /// Отправка команды зажатия/разжатия клешни
    /// </summary>
    public void PublishGripperCmd(int actionID)
    {
        Int32Msg msg = new Int32Msg { data = actionID };
        ros.Publish(cmdGripperTopic, msg);
    }

    /// <summary>
    /// Отправка угла поворота сервопривода камеры
    /// </summary>
    public void PublishCameraCmd(float angleDegrees)
    {
        Float32Msg msg = new Float32Msg { data = angleDegrees };
        ros.Publish(cmdCameraPanTopic, msg);
    }

    // ==========================================
    // МЕТОДЫ СОВМЕСТИМОСТИ ДЛЯ SimpleControl.cs
    // ==========================================

    /// <summary>
    /// Старый метод управления движением
    /// </summary>
    public void PublishCommand(float gas, float steer)
    {
        PublishCmdVel(gas, steer);
    }

    /// <summary>
    /// Старый метод управления манипулятором/клешней
    /// </summary>
    public void PublishArmCmd(int actionID)
    {
        PublishGripperCmd(actionID);
    }
}
