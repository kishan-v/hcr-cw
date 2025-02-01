using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using UnityEngine;

public class SRTReceiver : MonoBehaviour
{
    [Header("FFmpeg Settings")]
    public string ffmpegPath = "ffmpeg"; // Ensure ffmpeg is in your PATH or provide full path.
    public string serverIP = "192.168.1.100"; // Replace with your Theta’s IP
    public int serverPort = 9000;            // Replace with your stream’s port
    public int latency = 50;                 // Adjust latency if needed

    [Header("Video Stream Settings")]
    public int width = 1920;
    public int height = 1080;
    public int framerate = 30;
    public string pixelFormat = "bgr24";     // For raw output (3 bytes per pixel)
    
    private int frameSize;
    private Texture2D videoTexture;
    private Process ffmpegProcess;
    private Thread readThread;
    private byte[] frameBuffer;
    private volatile byte[] latestFrame = null;
    private readonly object frameLock = new object();
    private volatile bool isRunning = false;

    void Start()
    {
        // Calculate expected frame size (3 channels: B, G, R)
        frameSize = width * height * 3;

        // Create a Texture2D to hold the video frame data
        videoTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        // (Optionally, assign videoTexture to the material on your inverted sphere)
        Renderer renderer = GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.mainTexture = videoTexture;
        }

        // Start FFmpeg to decode the SRT stream
        StartFFmpeg();

        // Start a background thread to read frames from FFmpeg’s output
        isRunning = true;
        frameBuffer = new byte[frameSize];
        readThread = new Thread(ReadFrames);
        readThread.IsBackground = true;
        readThread.Start();
    }

    void StartFFmpeg()
    {
        ProcessStartInfo psi = new ProcessStartInfo
        {
            FileName = ffmpegPath,
            // Build the FFmpeg arguments:
            // - Input: SRT stream from Theta (modify query string as needed)
            // - Scale the stream to the desired width and height
            // - Set pixel format and output as raw video (pipe:1)
            Arguments = $"-y -fflags nobuffer -flags low_delay -strict experimental -thread_queue_size 512 -i \"srt://{serverIP}:{serverPort}?mode=caller&latency={latency}\" -vf scale={width}:{height} -pix_fmt {pixelFormat} -f rawvideo pipe:1",
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };

        ffmpegProcess = new Process { StartInfo = psi };
        ffmpegProcess.Start();

        // Optionally, read and log errors from FFmpeg in a separate thread:
        Thread errorThread = new Thread(() =>
        {
            StreamReader errReader = ffmpegProcess.StandardError;
            while (!errReader.EndOfStream)
            {
                string line = errReader.ReadLine();
                UnityEngine.Debug.Log("FFmpeg error: " + line);
            }
        });
        errorThread.IsBackground = true;
        errorThread.Start();
    }

    void ReadFrames()
    {
        Stream stdout = ffmpegProcess.StandardOutput.BaseStream;
        while (isRunning)
        {
            int offset = 0;
            while (offset < frameSize)
            {
                try
                {
                    int bytesRead = stdout.Read(frameBuffer, offset, frameSize - offset);
                    if (bytesRead == 0)
                    {
                        // End of stream or process exited.
                        isRunning = false;
                        break;
                    }
                    offset += bytesRead;
                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("Error reading frame: " + ex.Message);
                    isRunning = false;
                    break;
                }
            }
            if (offset == frameSize)
            {
                // Copy the frame into latestFrame so that Update() can process it.
                byte[] frameCopy = new byte[frameSize];
                Buffer.BlockCopy(frameBuffer, 0, frameCopy, 0, frameSize);
                lock (frameLock)
                {
                    latestFrame = frameCopy;
                }
            }
        }
    }

    void Update()
    {
        // Check if a new frame has been received.
        if (latestFrame != null)
        {
            lock (frameLock)
            {
                // Load raw frame data into the texture.
                videoTexture.LoadRawTextureData(latestFrame);
                videoTexture.Apply();
                latestFrame = null;
            }
        }
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        if (readThread != null && readThread.IsAlive)
        {
            readThread.Abort();
        }
        if (ffmpegProcess != null && !ffmpegProcess.HasExited)
        {
            ffmpegProcess.Kill();
        }
    }
}