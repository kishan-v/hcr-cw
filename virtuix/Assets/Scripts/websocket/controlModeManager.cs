using UnityEngine;

public enum ControlMode
{
    Virtuix,
    Joystick
}

public class ControlModeManager : MonoBehaviour
{
    public static ControlMode activeMode = ControlMode.Virtuix; // Default mode

    void Update()
    {
        // Toggle control mode when T is pressed.
        if (Input.GetKeyDown(KeyCode.T))
        {
            if (activeMode == ControlMode.Virtuix)
            {
                activeMode = ControlMode.Joystick;
                Debug.Log("Switched to Joystick mode");
            }
            else
            {
                activeMode = ControlMode.Virtuix;
                Debug.Log("Switched to Virtuix mode");
            }
        }
    }
}
