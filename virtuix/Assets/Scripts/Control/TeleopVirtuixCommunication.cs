using UnityEngine;
using static System.Math;
using System;
using Newtonsoft.Json;
using System.Collections.Concurrent;
using Unity.XR.CoreUtils;
using System.Runtime.Remoting.Messaging;
using System.Collections.Generic;


public class TeleopOmniCommunication : MonoBehaviour
{
    // Virtuix component (dummy movement component object)
    public OmniMovementComponent omniMovement;

    // MOVEMENT
    public float movementThreshold = 0.01f;         // Filter out small movements
    public float movementMultiplier = 10;           // Scale movement
    public Vector3 movement = new(0, 0, 0);         // Debug
    public Vector3 debugMovement = new(0, 0, 0);    // Debug
    public int noStepThreshold = 10;                // Number of 0 steps before a 'STOP' command is sent
    public float stepIncrement = 0.02f;             // Size of a single step
    public Vector3 speedLimit = new Vector3(0.4f, 0f, 0.4f);

    private Vector3 stepMultiplier;
    private Vector3 stepDivisor;
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

    void Start()
    {        
        // Initialize the OmniMovementComponent
        omniMovement = GetComponent<OmniMovementComponent>();
        if (omniMovement == null)
        {
            Debug.LogError("OmniMovementComponent not found!");
        }

        // Calculate movement parameters
        stepMultiplier = new Vector3(1/stepIncrement, 1f/stepIncrement, 1f/stepIncrement);
        stepDivisor = new Vector3(stepIncrement, stepIncrement, stepIncrement);
    }


    float DegToRad(float degrees)
    {
        // Convert degrees to radians
        double radians = degrees * (Math.PI / 180.0);
        // Normalize angle
        return (float)Math.IEEERemainder(radians, 2 * Math.PI);
    }

    Vector3 ApplyAbs(Vector3 movement)
    {
        movement.x = Mathf.Abs(movement.x);
        movement.y = Mathf.Abs(movement.y);
        movement.z = Mathf.Abs(movement.z);
        return movement;
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

    Vector3 ApplySpeedLimit(Vector3 movement)
    {
        Vector3 mangitude = new Vector3(Mathf.Abs(movement.x), Mathf.Abs(movement.y), Mathf.Abs(movement.z));
        Vector3 sign = new Vector3(Mathf.Sign(movement.x), Mathf.Sign(movement.y), Mathf.Sign(movement.z));
        return Vector3.Scale(Vector3.Min(mangitude, speedLimit), sign);
    }

    // Rotates sphere so forward of operator is always forward of the sphere
    void RotateSphereMatchVirtuix()
    {
        float diff = previousDegRotation - degRotation;
        sphere.Rotate(Vector3.up * diff);
        previousDegRotation = degRotation;
    }

    void FixedUpdate()
    {
        // Check in Virtuix mode
        if (ControlModeManager.activeMode != ControlMode.Virtuix)
            return;
            
        if (omniMovement != null)
        {
            // Update omni's data
            omniMovement.GetOmniInputForCharacterMovement();

            // MOVEMENT
            movement = omniMovement.GetForwardMovement() + omniMovement.GetStrafeMovement();
            debugMovement = movement;

            //movement = ApplyAbs(movement);
            movement = ApplyMovingAverage(movement);
            movement *= movementMultiplier;
            movement = ApplySteppedMovement(movement);
            movement = ApplySpeedLimit(movement);

            // Check movement above threshold
            if (Math.Abs(movement.x) > movementThreshold)
            {
                noStepCount = 0;
            }
            else
            {
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
            degRotation = omniMovement.currentOmniYaw;
            float radRotation = DegToRad(degRotation);

            // Check rotation above threshold
            if (Math.Abs((float)radRotation - previousRadRotation) > rotationThreshold)
            {
                rotateFlag = true;
                RotateSphereMatchVirtuix();
                previousRadRotation = radRotation;
            }
            else
            {
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
                        linear = new { x = -movement.x, y = 0.0, z = 0.0 },
                        angular = new { x = 0.0, y = 0.0, z = -radRotation },
                        timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds()
                    }
                };

                string message = JsonConvert.SerializeObject(command);
                try
                {
                    WebSocketController.Instance.SendMessageWebsocket(message);
                }
                catch (Exception ex)
                {
                    Debug.LogError("Error sending message: " + ex.Message);
                }
                previousMovement = movement;
            }
        }
    }
}
