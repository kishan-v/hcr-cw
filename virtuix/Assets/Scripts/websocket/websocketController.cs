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
    private ConcurrentQueue<string> lidarDataQueue = new ConcurrentQueue<string>();
    private bool shouldQuit = false;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            ConnectWebSocket();
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
            Debug.Log("Received message: " + e.Data);
            // If the queue is full (max 5 items), remove the oldest message.
            if(lidarDataQueue.Count >= 5)
            {
                string discarded;
                lidarDataQueue.TryDequeue(out discarded);
            }
            // Enqueue the new LiDAR data.
            lidarDataQueue.Enqueue(e.Data);
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

    public void SendMessage(string message)
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

    void Update()
    {
        // Process one LiDAR message per frame (or adjust as needed).
        if (lidarProcessor != null && lidarDataQueue.TryDequeue(out string lidarData))
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
