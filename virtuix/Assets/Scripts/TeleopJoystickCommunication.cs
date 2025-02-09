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

    // Deadzone threshold
    public float deadzone = 0.1f;

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
        
	// When joystick clicked 
        if (touch.GetState(handType))
        {
            // Get the joystick's trackpad touch position.
            Vector2 axis = joystick.GetAxis(handType);
            // Deadzone to avoid sending commands on minor joystick noise.
            if (axis.magnitude < deadzone)
            {
                axis = Vector2.zero;
            }

            // Map joystick Y to forward/backward and X to turning.
            double linear_x = axis.y;    // Forward/backward command
            double angular_z = axis.x;   // Turning command

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
