using UnityEngine;
using System;
using WebSocketSharp;
using Newtonsoft.Json;
using Valve.VR;

public class TeleopJoystickCommunication : MonoBehaviour
{

    // Which controller to use.
    public SteamVR_Input_Sources handType = SteamVR_Input_Sources.LeftHand;

    public SteamVR_Action_Vector2 joystick = SteamVR_Actions.default_Joystick;
    public SteamVR_Action_Boolean touch = SteamVR_Actions.default_JoystickTouch;

    public double movementMultiplier = 0.4;

    // Deadzone threshold
    public float deadzone = 0.1f;


    void Update()
    {

        if (ControlModeManager.activeMode != ControlMode.Joystick)
            return;

        // When joystick clicked 
        if (touch.GetState(handType))
        {
            // Get the joystick's trackpad touch position.
            Vector2 axis = joystick.GetAxis(handType);

            // Deadzone to avoid sending commands on minor joystick noise.
            if (axis.magnitude < deadzone)
            {
                return;
            }

            // Get joystick inputs
            double angular = axis.x;
            double linear = axis.y * movementMultiplier;

            var command = new
            {
                op = "command",
                topic = "teleop/cmd_vel",
                type = "joystick",
                msg = new
                {
                    linear = new { x = linear, y = 0.0, z = 0.0 },
                    angular = new { x = 0.0, y = 0.0, z = -angular },
                    timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds()
                }
            };

            string message = JsonConvert.SerializeObject(command);
            try
            {
                WebSocketController.Instance.SendMessage(message)
                Debug.Log("Sent command: " + message);
            }
            catch (Exception ex)
            {
                Debug.LogError("Error sending message: " + ex.Message);
            }
        }

        // Quit with Q
        if (Input.GetKeyDown(KeyCode.Q))
        {
            Debug.Log("Quitting...");
            // shouldQuit = true;
            // ws.Close();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }
    }

}
