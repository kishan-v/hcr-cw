using UnityEngine;

public enum ControlMode
{
    Virtuix,
    Joystick,

    None
}

public class ControlModeManager : MonoBehaviour
{
    [SerializeField]
    public static ControlMode activeMode = ControlMode.None; // Default mode

    void Update()
    {
        buttonPressed =
        // Toggle control mode when T is pressed.
        if (Input.GetKeyDown(KeyCode.X))
        {
            activeMode = ControlMode.None;
            Debug.Log("Switched to NO TRANSMIT")

        }
        if (Input.GetKeyDown(KeyCode.V))
        {
            activeMode = ControlMode.Virtuix;
            Debug.Log("Switched to Virtuix mode");

        }
        if (Input.GetKeyDown(KeyCode.J))
        {
            activeMode = ControlMode.Joystick;
            Debug.Log("Switched to Joystick mode");

        }
        
    }
}
