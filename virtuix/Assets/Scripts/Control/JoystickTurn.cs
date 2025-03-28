using UnityEngine;
using Valve.VR;

public class JoystickSphere : MonoBehaviour
{
    // Assign in inspector
    public SteamVR_Input_Sources handType; 
    public SteamVR_Action_Boolean trigger;
    public Transform sphere;
    public float rotationSpeed = 45f; // Degrees per second

    void Start()
    {
        if (handType == SteamVR_Input_Sources.RightHand) 
        {
            rotationSpeed = -rotationSpeed;
        }
        else if (handType != SteamVR_Input_Sources.LeftHand)
        {
            Debug.LogError("Invalid hand type");
        }
    }
    
    void Update()
    {
        // Trigger pressed
        if (trigger.GetState(handType))
        {
            // Rotate the sphere around its Y axis at constant speed
            sphere.Rotate(rotationSpeed * Time.deltaTime * Vector3.up);
        }
    }
}
