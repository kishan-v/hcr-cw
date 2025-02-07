using UnityEngine;
using System;
using WebSocketSharp;
using Newtonsoft.Json;

public class TeleopCommunication : MonoBehaviour
{
    private WebSocket ws;
    private bool shouldQuit = false;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";

    void Start()
    {
        ConnectWebSocket();
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
            Debug.Log("Received reply: " + e.Data);
        };

        ws.OnError += (sender, e) =>
        {
            Debug.LogError("Error: " + e.Message);
        };

        ws.OnClose += (sender, e) =>
        {
            Debug.Log($"Connection closed. Code: {e.Code} Reason: {e.Reason}");
            if (!shouldQuit)
            {
                // Try to reconnect after a delay.
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

    void Update()
    {
        if (ws == null || !ws.IsAlive)
        {
            return;
        }

        // Use Unity's Input class to poll key presses.
        double linear_x = 0.0;
        double angular_z = 0.0;
        bool sendCommand = false;

        if (Input.GetKeyDown(KeyCode.W))
        {
            linear_x = 1.0;
            sendCommand = true;
        }
        else if (Input.GetKeyDown(KeyCode.S))
        {
            linear_x = -1.0;
            sendCommand = true;
        }
        else if (Input.GetKeyDown(KeyCode.A))
        {
            angular_z = 1.0;
            sendCommand = true;
        }
        else if (Input.GetKeyDown(KeyCode.D))
        {
            angular_z = -1.0;
            sendCommand = true;
        }
        else if (Input.GetKeyDown(KeyCode.X))
        {
            // Stop command (both values 0)
            linear_x = 0.0;
            angular_z = 0.0;
            sendCommand = true;
        }
        else if (Input.GetKeyDown(KeyCode.Q))
        {
            // Quit the connection and exit play mode.
            Debug.Log("Quitting...");
            shouldQuit = true;
            ws.Close();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }

        if (sendCommand)
        {
            var command = new
            {
                op = "command",
                topic = "teleop/cmd_vel",
                msg = new
                {
                    linear = new { x = linear_x, y = 0.0, z = 0.0 },
                    angular = new { x = 0.0, y = 0.0, z = angular_z },
                    timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds()
                }
            };

            string message = JsonConvert.SerializeObject(command);
            try
            {
                ws.Send(message);
                Debug.Log("Sent command: " + message);
            }
            catch (Exception ex)
            {
                Debug.LogError("Error sending message: " + ex.Message);
            }
        }
    }

    private void OnDestroy()
    {
        shouldQuit = true;
        if (ws != null)
        {
            ws.Close();
        }
    }
}