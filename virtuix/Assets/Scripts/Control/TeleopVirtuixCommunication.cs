using UnityEngine;
using static System.Math;
using System;
using WebSocketSharp;
using Newtonsoft.Json;
using System.Collections.Concurrent;
using Unity.XR.CoreUtils;
using System.Runtime.Remoting.Messaging;
using System.Collections.Generic;

public class TeleopOmniCommunication : MonoBehaviour
{
    private WebSocket ws;
    private bool shouldQuit = false;
    private const string RELAYER_URL = "ws://132.145.67.221:9090";

    // Virtuix component (dummy movement component object)
    public OmniMovementComponent omniMovement;

    // MOVEMENT
    public float movementThreshold = 0.01f;         // Filter out small movements
    public float movementMultiplier = 10;           // Scale movement
    public Vector3 movement = new(0, 0, 0);         // Debug
    public Vector3 debugMovement = new(0, 0, 0);    // Debug
    public int noStepThreshold = 10;                // Number of 0 steps before a 'STOP' command is sent
    public double speedLimit = 0.4;                 // Maximum output speed
    public float stepIncrement = 0.02f;             // Size of a single step

    private Vector3 stepMultiplier;
    private Vector3 stepDivisor;
    private Vector3 speedLimitVec;
    private Vector3 previousMovement = new(0, 0, 0);
    private int noStepCount = 0;

    // Moving average configuration
    public int movingAverageWindow = 10;            // Number of frames to average over
    private Queue<Vector3> movementHistory = new Queue<Vector3>();
    private Vector3 movingAverageSum = Vector3.zero;

    // ROTATION
    private float previousRadRotation = 0;             // Most recent sent rotation (Radians)
    private float previousDegRotation = 0;        // Previous rotation to turn sphere (Degrees)  
    public float rotationThreshold = 0.2f;
    
    public Transform sphere;
    private float degRotation = 0;
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

        // Calculate movement parameters
        stepMultiplier = new Vector3(1/stepIncrement, 1f/stepIncrement, 1f/stepIncrement);
        stepDivisor = new Vector3(stepIncrement, stepIncrement, stepIncrement);
        speedLimitVec = Vector3.one * (float)speedLimit;
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

    float DegToRad(float degrees)
    {
        // Convert degrees to radians
        double radians = degrees * (Math.PI / 180.0);
        // Normalize angle
        return (float)Math.IEEERemainder(radians, 2 * Math.PI);
    }

    float RadToDeg(float radians)
    {
        // Convert degrees to radians
        double degrees = radians * (180 / Math.PI);
        // Normalize angle
        return (float)Math.IEEERemainder(degrees, 360);
    }

    Vector3 ApplyMovingAverage(Vector3 newMovement)
    {
        // Add new movement to history
        movementHistory.Enqueue(newMovement);
        movingAverageSum += newMovement;
        
        // Remove oldest entry if over window size
        if (movementHistory.Count > movingAverageWindow)
        {
            Vector3 oldest = movementHistory.Dequeue();
            movingAverageSum -= oldest;
        }
        
        return movingAverageSum / movementHistory.Count;
    }

    Vector3 ApplySteppedMovement(Vector3 movement)
    {
        // Apply stepping
        Vector3 stepped = Vector3.Scale(movement, stepMultiplier);
        stepped.x = Mathf.Round(stepped.x);
        stepped.y = Mathf.Round(stepped.y);
        stepped.z = Mathf.Round(stepped.z);
        return Vector3.Scale(stepped, stepDivisor);
    }

    // Rotates sphere so perceived video matches the rotations of dog (delay)
    // TODO: requires getting the dog's rotation
    void RotateSphereMatchDog()
    {

    }

    // Rotates sphere so perceived video matches operator's turning
    // Instantly rotates
    void RotateSphereMatchVirtuix()
    {
        float diff = degRotation - previousDegRotation;
        sphere.Rotate(Vector3.up * diff);
        previousDegRotation = degRotation;
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
            // Update omni's data
            omniMovement.GetOmniInputForCharacterMovement();

            // MOVEMENT
            movement = omniMovement.GetForwardMovement() + omniMovement.GetStrafeMovement();
            debugMovement = movement;

            movement = ApplyMovingAverage(movement);
            movement = ApplySteppedMovement(movement);
            movement *= movementMultiplier;                     // Apply multiplier
            movement = Vector3.Min(movement, speedLimitVec);    // Apply speed limit

            // Check movement above threshold
            if (Math.Abs(movement.x) > movementThreshold) 
            {
                noStepCount = 0;
            }
            else {
                noStepCount += 1;
                if (noStepCount > noStepThreshold)
                {
                    movement = Vector3.zero;
                }
                else
                {
                    movement = previousMovement;
                }
            }

            // ROTATION
            float degRotation = omniMovement.currentOmniYaw;
            float radRotation = DegToRad(degRotation);

            // Check rotation above threshold
            if (Math.Abs((float)radRotation - previousRadRotation) > rotationThreshold) {
                rotateFlag = true;
                RotateSphereMatchVirtuix();
                previousRadRotation = radRotation;
            }
            else {
                rotateFlag = false;
                radRotation = previousRadRotation;
            }

            // Send message to dog if change
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
                        angular = new { x = 0.0, y = 0.0, z = -radRotation },
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
