using UnityEngine;
using Valve.VR;

public class HapticCollision : MonoBehaviour
{
    public SteamVR_Action_Vibration hapticAction;
    public SteamVR_Input_Sources handType;

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("HapticCube")) // Ensure cube has this tag
        {
            TriggerHapticFeedback(1.0f, 0.1f, 80);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("HapticCube")) // Ensure cube has this tag
        {        
            //Debug.Log("Trigger " + gameObject.name + " with " + other.gameObject.name);

            TriggerHapticFeedback(1.0f, 0.1f, 80);
        }
    }

    private void TriggerHapticFeedback(float amplitude, float duration, float frequency)
    {

        if (hapticAction != null)
        {
            hapticAction.Execute(0, duration, frequency, amplitude, handType);
        }
    }
}
