using UnityEngine;
using UnityEngine.Video;

public class VideoManager : MonoBehaviour
{

    private VideoPlayer videoPlayer;

    private void Start()
    {
        videoPlayer = GetComponent<VideoPlayer>();
    }

    // Update is called once per frame
    public void Play()
    {
        videoPlayer.Play();
    }

    // Update is called once per frame
    public void Pause()
    {
        videoPlayer.Pause();
    }

    // Update is called once per frame
    public void Stop()
    {
        videoPlayer.Stop();
    }

    public void URLToVideo(string url)
    {
        videoPlayer.source = VideoSource.Url;
        videoPlayer.url = url;
        videoPlayer.Prepare();
        videoPlayer.prepareCompleted += VideoPlayer_prepareCompleted;
    }

    private void VideoPlayer_prepareCompleted(VideoPlayer source)
    {
        Play();
    }
}
