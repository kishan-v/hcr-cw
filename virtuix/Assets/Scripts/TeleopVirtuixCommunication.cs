using UnityEngine;
using System;
using WebSocketSharp;
using Newtonsoft.Json;

public class TeleopOmniCommunication : MonoBehaviour
{
    private WebSocket ws;
    private bool shouldQuit = false;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";

    // Reference to the Virtuix Omni movement component.
    // This component must be part of your scene (typically on your OmniCharacterController prefab).
    public OmniMovementComponent omniMovement;

    // A threshold below which we treat movement as zero (to filter out noise).
    public float movementThreshold = 0.05f;

    void Start()
    {
        ConnectWebSocket();
        
        // Initialize the OmniMovementComponent
        omniMovement = GetComponent<OmniMovementComponent>();
        if (omniMovement == null)
        {
            Debug.LogError("OmniMovementComponent not found!");
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
            return;

        if (omniMovement != null)
        {
            // Ensure the Omni movement component updates its internal data.
            // (If you’re in developerMode, the OmniMovementComponent may use alternate input.
            // Otherwise, it automatically reads from the treadmill.)
            omniMovement.GetOmniInputForCharacterMovement();

            // Retrieve the calculated movement vectors.
            Vector3 forwardMovement = omniMovement.GetForwardMovement();
            Vector3 strafeMovement = omniMovement.GetStrafeMovement();
	    float rotation = omniMovement.currentOmniYaw;
            Debug.Log(rotation);

            // For this example, we map:
            // • Forward/backward speed from the forwardMovement's z value.
            // • Turning (angular) speed from the strafeMovement's x value.
            double linearCommand = forwardMovement.z;
            double angularCommand = strafeMovement.x;

            // Apply a deadzone so we only send meaningful commands.
            if (Mathf.Abs((float)linearCommand) < movementThreshold)
                linearCommand = 0.0;
            if (Mathf.Abs((float)angularCommand) < movementThreshold)
                angularCommand = 0.0;

            // Build the command payload.
            var command = new
            {
                op = "command",
                topic = "teleop/cmd_vel",
                msg = new
                {
                    // In this protocol, we assume the linear motion is along the x-axis.
                    // Adjust the mapping as needed (e.g. swap axes) to suit your application.
                    linear = new { x = linearCommand, y = 0.0, z = 0.0 },
                    angular = new { x = 0.0, y = 0.0, z = angularCommand },
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

        // Optionally, allow quitting via keyboard (for testing).
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
