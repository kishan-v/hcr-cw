using UnityEngine;
using System.IO;
using System;
using Valve.VR;
using WebSocketSharp;
using Newtonsoft.Json;

public class VirtuixRecord : MonoBehaviour
{
    private StreamWriter csvWriter;
    private readonly string csvFile = "VirtuixData.csv";

    public OmniMovementComponent omniMovement;

    void Start()
    {
        // Initialize the OmniMovementComponent
        omniMovement = GetComponent<OmniMovementComponent>();
        if (omniMovement == null)
        {
            Debug.LogError("OmniMovementComponent not found!");
        }

        csvWriter = new StreamWriter(csvFile, false);
        csvWriter.WriteLine("Timestamp,Forward.x,Forward.y,Forward.z,Strafe.x,Strafe.y,Strafe.z,Angle");
    }

    void Update()
    {
        omniMovement.GetOmniInputForCharacterMovement();

        // Write to CSV file
        long timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
        Vector3 forward = omniMovement.GetForwardMovement();
        Vector3 strafe = omniMovement.GetStrafeMovement();
        float rotation = omniMovement.currentOmniYaw;
        csvWriter.WriteLine($"{timestamp},{forward.x},{forward.y},{forward.z},{strafe.x},{strafe.y},{strafe.z},{rotation}");
    }

    void OnDestroy()
    {
        if (csvWriter != null)
        {
            csvWriter.Flush();
            csvWriter.Close();
            csvWriter = null;
        }
    }
}
