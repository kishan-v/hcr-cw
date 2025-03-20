using UnityEngine;
using WebSocketSharp;
using System;
using System.Collections.Concurrent;

public class WebSocketController : MonoBehaviour
{
    public static WebSocketController Instance { get; private set; }
    private WebSocket ws;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";
    public LidarProcessor lidarProcessor;

    // Queue to store incoming LiDAR messages.
    private ConcurrentQueue<byte[]> lidarDataQueue = new ConcurrentQueue<byte[]>();
    private bool shouldQuit = false;

    private float totalRTT = 0f;
    private int countRTT = 0;

    [Serializable]
    public class RTTMessage
    {
        public string type = "RTT";
        public string timestamp;
    }


    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            ConnectWebSocket();
            StartCoroutine(SendRTTMessages());
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void ConnectWebSocket()
    {
        ws = new WebSocket(RELAYER_URL);

        ws.OnOpen += (sender, e) =>
        {
            Debug.Log("Connected to server.");
        };

        ws.OnMessage += (sender, e) =>
        {
            //Debug.Log("Received message: " + e.Data);
            //If the queue is full(max 5 items), remove the oldest message.
            //  if (lidarDataQueue.Count >= 5)
            // {
            //     Debug.Log("Discarding Lidar");
            //     byte[] discarded;
            //     lidarDataQueue.TryDequeue(out discarded);
            // }
            // // Enqueue the new LiDAR data.
            // lidarDataQueue.Enqueue(e.RawData);

            try
            {
                RTTMessage rttMsg = JsonUtility.FromJson<RTTMessage>(e.Data);
                if (rttMsg != null && rttMsg.type == "RTT")
                {
                    DateTime sentTime = DateTime.Parse(rttMsg.timestamp, null, System.Globalization.DateTimeStyles.RoundtripKind);
                    double rtt = (DateTime.UtcNow - sentTime).TotalMilliseconds;
                    totalRTT += (float)rtt;
                    countRTT++;
                    float averageRTT = totalRTT / countRTT;
                    Debug.Log($"Received RTT: {rtt} ms, Average RTT: {averageRTT} ms");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError("Failed to parse RTT message: " + ex.Message);
            }
            //lidarProcessor.ProcessLidarData(e.RawData);
        };

        ws.OnError += (sender, e) =>
        {
            Debug.LogError("WebSocket Error: " + e.Message);
        };

        ws.OnClose += (sender, e) =>
        {
            Debug.Log($"WebSocket closed: Code {e.Code} Reason: {e.Reason}");
            if (!shouldQuit)
            {
                Invoke("ConnectWebSocket", 5f);
            }
        };

        try
        {
            ws.ConnectAsync();
        }
        catch (Exception ex)
        {
            Debug.LogError("Exception during connect: " + ex.Message);
        }
    }


    public void SendMessageWebsocket(string message)
    {
        if (ws != null && ws.IsAlive)
        {
            ws.Send(message);
            Debug.Log("Sent message: " + message);
        }
        else
        {
            Debug.LogWarning("WebSocket not connected.");
        }
    }

    IEnumerator SendRTTMessages()
    {
        while (true)
        {
            RTTMessage msg = new RTTMessage();
            // Use ISO 8601 for the timestamp (UTC time)
            msg.timestamp = DateTime.UtcNow.ToString("o");
            string jsonMessage = JsonUtility.ToJson(msg);
            SendMessageWebsocket(jsonMessage);
            yield return new WaitForSeconds(1f);
        }
    }

    void Update()
    {
        // Process one LiDAR message per frame
        if (lidarProcessor != null && lidarDataQueue.TryDequeue(out byte[] lidarData))
        {
            lidarProcessor.ProcessLidarData(lidarData);
        }
    }

    void OnDestroy()
    {
        shouldQuit = true;
        if (ws != null)
        {
            ws.Close();
        }
    }
}
