// Author: Kishan <kishan-v@users.noreply.github.com>

using UnityEngine;

public class Video360Manager : MonoBehaviour
{
    public WebRTCReceiver webrtcReceiver;
    private Material sphereMaterial;

    void Start()
    {
        // Get the material from the Renderer component
        sphereMaterial = GetComponent<Renderer>().material;
        webrtcReceiver.OnVideoTextureUpdated += OnVideoTextureUpdated;
    }

    private void OnVideoTextureUpdated(Texture texture)
    {
        Debug.Log("Video texture updated");
        if (sphereMaterial != null)
        {
            sphereMaterial.mainTexture = texture;
        }
    }
}