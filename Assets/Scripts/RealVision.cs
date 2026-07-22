using UnityEngine;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Collections.Concurrent;

[System.Serializable]
public class YoloDataPacket
{
    public float angle;      // Отклонение центра мяча (-1.0 лево, 1.0 право)[cite: 2]
    public float distance;   // Высота рамки мяча относительно кадра (0..1)[cite: 2]
    public float sees;       // Флаг видимости (1.0 = виден, 0.0 = нет)[cite: 2]
    public float conf;       // Уверенность детекции[cite: 2]
    public float w;          // Ширина bounding box[cite: 2]
    public float h;          // Высота bounding box[cite: 2]
}

public class RealVision : MonoBehaviour
{
    public int udpPort = 5005; //[cite: 2]
    public bool useYOLO = false; //[cite: 2]
    
    // Телеметрия для RobotBrain[cite: 2]
    public float normalizedAngle; //[cite: 2]
    public float normalizedDistance; //[cite: 2]
    public bool seesBall; //[cite: 2]

    private CancellationTokenSource cts; //[cite: 2]
    private ConcurrentQueue<YoloDataPacket> udpQueue = new ConcurrentQueue<YoloDataPacket>(); //[cite: 2]

    void Start()
    {
        cts = new CancellationTokenSource(); //[cite: 2]
        // Запуск UDP-слушателя в фоновом потоке, чтобы не вешать игру[cite: 2]
        Task.Run(() => UdpListenerLoop(cts.Token)); //[cite: 2]
    }

    private async Task UdpListenerLoop(CancellationToken token)
    {
        using (var udpClient = new UdpClient(udpPort)) //[cite: 2]
        {
            while (!token.IsCancellationRequested) //[cite: 2]
            {
                var result = await udpClient.ReceiveAsync(); //[cite: 2]
                string json = System.Text.Encoding.UTF8.GetString(result.Buffer); //[cite: 2]
                
                YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json); //[cite: 2]
                if (packet != null) //[cite: 2]
                {
                    udpQueue.Enqueue(packet); // Безопасно кладем в очередь[cite: 2]
                }
            }
        }
    }

    void Update()
    {
        // Читаем пакеты из очереди на главном потоке Unity[cite: 2]
        while (udpQueue.TryDequeue(out var packet)) //[cite: 2]
        {
            useYOLO = true; //[cite: 2]
            seesBall = packet.sees > 0.5f; //[cite: 2]

            if (seesBall) //[cite: 2]
            {
                // Ограничиваем угол [-1, 1][cite: 2]
                normalizedAngle = Mathf.Clamp(packet.angle, -1f, 1f); //[cite: 2]
                
                // Записываем нормализованную высоту рамки (чем больше мяч, тем он ближе)[cite: 2]
                normalizedDistance = packet.distance;  //[cite: 2]
            }
            else //[cite: 2]
            {
                normalizedAngle = 0f; //[cite: 2]
                normalizedDistance = 1f; // 1.0 = далеко/мяч не виден[cite: 2]
            }
        }
    }

    void OnDestroy()
    {
        cts?.Cancel(); // Останавливаем фоновый поток при выходе[cite: 2]
    }
}