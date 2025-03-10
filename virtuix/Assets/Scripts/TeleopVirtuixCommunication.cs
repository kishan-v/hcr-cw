using UnityEngine;
using static System.Math;
using System;
using WebSocketSharp;
using System.Text;
using Newtonsoft.Json;
using System.Collections.Concurrent;
using Unity.XR.CoreUtils;
using System.Runtime.Remoting.Messaging;

public class TeleopOmniCommunication : MonoBehaviour
{
    private WebSocket ws;
    private bool shouldQuit = false;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";

    // Reference to the Virtuix Omni movement component.
    // This component must be part of your scene (typically on your OmniCharacterController prefab).
    public OmniMovementComponent omniMovement;

    // A threshold below which we treat movement as zero (to filter out noise).
    public float movementThreshold = 0.01f;
    public float movementMultiplier = 10;
    private Vector3 previousMovement = new(0, 0, 0);
    public Vector3 movement = new(0, 0, 0); // Debug
    public Vector3 debugMovement = new(0, 0, 0); // Debug
    private int noStepCount = 0;
    public int noStepThreshold = 10;

    public double speedLimit = 0.4;
    private int steps = 5;

    private float previousRotation = 0;
    public float rotationThreshold = 0.2f;
    private bool rotateFlag = true;

    [SerializeField]
    private LidarProcessor lidarProcessor;

    // concurrent queue to store messages
    private ConcurrentQueue<byte[]> lidarDataQueue = new ConcurrentQueue<byte[]>();

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
            lidarDataQueue.Enqueue(e.RawData);
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

    private Vector3 multiplier = new Vector3(5, 5, 5);
    private Vector3 divisor = new Vector3(1/5, 1/5, 1/5);
    private Vector3 speedLimitVec = new Vector3(2/8, 2/8, 2/8);

    Vector3 StepVector(Vector3 vec)
    {
        // Scale
        vec = Vector3.Scale(vec, multiplier);

        // Round
        vec.x = (float)Math.Round(vec.x);
        vec.y = (float)Math.Round(vec.y);
        vec.z = (float)Math.Round(vec.z);

        // Divide
        vec = Vector3.Scale(vec, divisor);

        // Apply movement multiplier
        //vec = Vector3.Scale(vec, )

        // Apply speed limit
        vec = Vector3.Max(vec, speedLimitVec);

        return vec;
    }



    void Update()
    {
        // LIDAR
        while (lidarDataQueue.TryDequeue(out byte[] lidarBytes))
        {
            if (lidarProcessor != null)
            {
                lidarProcessor.ProcessLidarData(lidarBytes);
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
            movement = omniMovement.GetForwardMovement() + omniMovement.GetStrafeMovement();
            debugMovement = movement;

            if (Math.Abs(movement.x) > movementThreshold) {
                noStepCount = 0;
                movement = new Vector3((float)-0.4,(float)0.0,(float)0.0);
                //double steppedLinear = Math.Round(debugMovement.Multiply(Vector3.(steps,steps,steps))) / steps;
                //steppedLinear = Vector3.Max(steppedLinear, speedLimit);
                //movement = new Vector3((float)steppedLinear, 0, 0);
            }
            else {
                noStepCount += 1;
                if (noStepCount > noStepThreshold)
                {
                    movement = new Vector3((float)0.0, (float)0.0, (float)0.0);
                }
                else
                {
                    movement = previousMovement;
                }
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

            //Debug.Log("Movement in the x after filtering " + movement.x);
            // Send message to dog
            if (rotateFlag || (movement != previousMovement))
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
            previousMovement = movement;
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
