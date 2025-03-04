using UnityEngine;
using static System.Math;
using System;
using WebSocketSharp;
using Newtonsoft.Json;
using System.Collections.Concurrent;

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
    public float movementMultiplier = 10;
    private int noStepCount = 0;
    public int noStepThreshold = 3;

    public double speedLimit = 0.4;
    //private Vector3 movementLimit = new Vector3(speedLimit, speedLimit, speedLimit);
    public int steps = 5;

    private float previousRotation = 0;
    public float rotationThreshold = 0.2f;
    private bool rotateFlag = true;

    [SerializeField]
    private LidarProcessor lidarProcessor;

    // concurrent queue to store messages
    private ConcurrentQueue<string> lidarDataQueue = new ConcurrentQueue<string>();


    void Start()
    {
        ConnectWebSocket();
        
        // Initialize the OmniMovementComponent
        omniMovement = GetComponent<OmniMovementComponent>();
        if (omniMovement == null)
        {
            Debug.LogError("OmniMovementComponent not found!");
        }

        if (lidarProcessor == null)
        {
            Debug.LogError("LidarProcessor not found!");
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
            //Debug.Log("Received reply: " + e.Data);
            //var messageObj = JsonConvert.DeserializeObject<dynamic>(e.Data);

            //if (messageObj != null && messageObj.world_dims != null)
            //{
            //    Debug.Log("Enqueuing LiDAR message.");
            //    lidarDataQueue.Enqueue(e.Data);
            //}
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

    float degToRad(float degrees)
    {
        // Convert degrees to radians
        double radians = degrees * (Math.PI / 180.0);
        // Normalize angle
        return (float)Math.IEEERemainder(radians, 2 * Math.PI);
    }

    

    void Update()
    {
        // LIDAR
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
            return;

        if (omniMovement != null)
        {
            // Ensure the Omni movement component updates its internal data.
            omniMovement.GetOmniInputForCharacterMovement();

            // Get movement
            Vector3 movement = omniMovement.GetForwardMovement(); // + omniMovement.GetStrafeMovement();
            Debug.Log(omniMovement.GetForwardMovement());
            //Debug.Log(omniMovement.GetStrafeMovement());


            if (movement.x < movementThreshold) {
                noStepCount = 0;
                //movement = new Vector3((float)-0.4,(float)0.0,(float)0.0);
                double steppedLinear = Math.Round(movement.x * steps) / steps;
                steppedLinear = Max(steppedLinear, speedLimit);
                movement = new Vector3((float)steppedLinear, 0, 0);
            }
            else {
                noStepCount += 1;
                movement = new Vector3((float)0.0, (float)0.0, (float)0.0);
            }

            // Apply movement multiplier
            movement *= movementMultiplier;
            // TODO: move speed limit after multip-lier
            //movement = Vector3.Min(movement, movementLimit);

            // Get rotation
            float radiansRotation = degToRad(omniMovement.currentOmniYaw);
            // Don't update rotation if it's below the threshold
            if (Math.Abs((float)radiansRotation - previousRotation) > rotationThreshold) {
                rotateFlag = true;
                previousRotation = radiansRotation;
            }
            else {
                rotateFlag = false;
                radiansRotation = previousRotation;
            }

            Debug.Log("Movement in the x after filtering " + movement.x);
            // Send message to dog
            if (rotateFlag || (noStepCount < noStepThreshold))
            {
                // Build the command payload.
                var command = new
                {
                    op = "command",
                    topic = "teleop/cmd_vel",
                    type = "omni",
                    msg = new
                    {
                        // In this protocol, we assume the linear motion is along the x-axis.
                        // Adjust the mapping as needed (e.g. swap axes) to suit your application.
                        linear = new { x = -movement.x, y = 0.0, z = 0.0 },
                        angular = new { x = 0.0, y = 0.0, z = -radiansRotation },
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
