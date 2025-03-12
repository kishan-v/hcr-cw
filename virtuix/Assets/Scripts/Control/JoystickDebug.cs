using UnityEngine;
using Valve.VR;

public class JoystickDebug : MonoBehaviour
{
    public SteamVR_Input_Sources handType; // Assign in inspector

    // Assign in Unity > Window > `SteamVR Input`
    public SteamVR_Action_Vector2 joystick = SteamVR_Actions.default_Joystick;
    public SteamVR_Action_Boolean touch = SteamVR_Actions.default_JoystickTouch;

    void Update()
    {
        // Check if trackpad is being touched
        if (touch.GetState(handType))
        {
            // Get trackpad coordinates
            Vector2 coordinates = joystick.GetAxis(handType);
            Debug.Log($"Trackpad Touch: X={coordinates.x}, Y={coordinates.y}");
        }
    }
}