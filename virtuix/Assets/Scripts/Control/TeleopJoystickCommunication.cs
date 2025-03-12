using UnityEngine;
using System;
using WebSocketSharp;
using Newtonsoft.Json;
using Valve.VR;

public class TeleopJoystickCommunication : MonoBehaviour
{
    private WebSocket ws;
    private bool shouldQuit = false;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";

    // Which controller to use.
    public SteamVR_Input_Sources handType = SteamVR_Input_Sources.LeftHand;

    public SteamVR_Action_Vector2 joystick = SteamVR_Actions.default_Joystick;
    public SteamVR_Action_Boolean touch = SteamVR_Actions.default_JoystickTouch;

    public double movementMultiplier = 0.4;

    // Deadzone threshold
    public float deadzone = 0.1f;

    [SerializeField]
    private LidarProcessor lidarProcessor;

    //TODO make the queue a fixed size (drop oldest)
    private ConcurrentQueue<string> lidarDataQueue = new ConcurrentQueue<string>();


    void Start()
    {
        if (lidarProcessor == null)
        {
            Debug.LogError("LidarProcessor not found!");
        }

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
            //Debug.Log("Received reply: " + e.Data);
            lidarDataQueue.Enqueue(e.Data);
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
        
        while (lidarDataQueue.TryDequeue(out string lidarString))
        {
            if (lidarProcessor != null)
            {
                lidarProcessor.ProcessLidarData(lidarString);
            }
            else
            {
                Debug.LogWarning("LidarProcessor not set. LiDAR data not processed.");
            }
        }

        if (ws == null || !ws.IsAlive)
        {
            return;
        }

        // When joystick clicked 
        if (touch.GetState(handType))
        {
            // Get the joystick's trackpad touch position.
            Vector2 axis = joystick.GetAxis(handType);
            
            // Deadzone to avoid sending commands on minor joystick noise.
            if (axis.magnitude < deadzone)
            {
                return;
            }

            // Get joystick inputs
            double angular = axis.x;
            double linear = axis.y * movementMultiplier;

            var command = new
            {
                op = "command",
                topic = "teleop/cmd_vel",
                type = "joystick",
                msg = new
                {
                    linear = new { x = linear, y = 0.0, z = 0.0 },
                    angular = new { x = 0.0, y = 0.0, z = -angular },
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

        // Quit with Q
        if (Input.GetKeyDown(KeyCode.Q))
        {
            Debug.Log("Quitting...");
            shouldQuit = true;
            ws.Close();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
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
