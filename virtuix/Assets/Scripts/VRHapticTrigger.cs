using UnityEngine;
using Valve.VR;

public class VRHapticTrigger : MonoBehaviour
{
    public SteamVR_Input_Sources handType = SteamVR_Input_Sources.LeftHand;
    

    public SteamVR_Action_Vibration hapticAction = SteamVR_Actions.default_Haptic;


    private void OnTriggerEnter(Collider other)
    {

        if (other.CompareTag("Controller"))
        {
    
            hapticAction.Execute(0, 0.1f, 150, 0.75f, handType);
        }
    }
}
