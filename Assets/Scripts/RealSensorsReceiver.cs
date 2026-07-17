using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Стандартные ROS-сообщения

public class RealSensorsReceiver : MonoBehaviour
{
    [Header("Hardware Sensor Topics")]
    public string ultrasoundTopic = "/sensor/ultrasound";
    public string leftIrTopic = "/sensor/ir_left";
    public string rightIrTopic = "/sensor/ir_right";
    public string gripperIrTopic = "/sensor/ir_gripper";

    [Header("YOLO Detection Topics")]
    public string ballVisibleTopic = "/yolo/ball_visible";
    public string ballAngleTopic = "/yolo/ball_angle";
    public string ballDistanceTopic = "/yolo/ball_distance";

    [Header("Current Sensor States (From Robot)")]
    public float ultrasoundValue = 1.0f; // Ожидается значение [0, 1]
    public int leftIRObstacle = 0;       // 0 или 1
    public int rightIRObstacle = 0;      // 0 или 1
    public bool isBallInGripper = false;

    [Header("Current YOLO States (From Robot)")]
    public bool isBallVisible = false;
    public float ballNormalizedAngle = 0.0f;    // Ожидается [-1, 1]
    public float ballNormalizedDistance = 1.0f; // Ожидается [0, 1]

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Подписываемся на аппаратные датчики робота
        ros.Subscribe<Float32Msg>(ultrasoundTopic, OnUltrasoundReceived);
        ros.Subscribe<Int32Msg>(leftIrTopic, OnLeftIrReceived);
        ros.Subscribe<Int32Msg>(rightIrTopic, OnRightIrReceived);
        ros.Subscribe<BoolMsg>(gripperIrTopic, OnGripperIrReceived);

        // Подписываемся на топики компьютерного зрения (YOLO)
        ros.Subscribe<BoolMsg>(ballVisibleTopic, OnBallVisibleReceived);
        ros.Subscribe<Float32Msg>(ballAngleTopic, OnBallAngleReceived);
        ros.Subscribe<Float32Msg>(ballDistanceTopic, OnBallDistanceReceived);
    }

    private void OnUltrasoundReceived(Float32Msg msg) { ultrasoundValue = msg.data; }
    private void OnLeftIrReceived(Int32Msg msg) { leftIRObstacle = msg.data; }
    private void OnRightIrReceived(Int32Msg msg) { rightIRObstacle = msg.data; }
    private void OnGripperIrReceived(BoolMsg msg) { isBallInGripper = msg.data; }

    private void OnBallVisibleReceived(BoolMsg msg) { isBallVisible = msg.data; }
    private void OnBallAngleReceived(Float32Msg msg) { ballNormalizedAngle = msg.data; }
    private void OnBallDistanceReceived(Float32Msg msg) { ballNormalizedDistance = msg.data; }
}