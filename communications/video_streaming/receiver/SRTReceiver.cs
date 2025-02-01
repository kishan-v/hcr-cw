using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using UnityEngine;

public class SRTReceiver : MonoBehaviour
{
    [Header("FFmpeg Settings")]
    public string ffmpegPath = "ffmpeg"; // Full path if not in system PATH.
    public string serverIP = "192.168.1.100"; // Your Theta's IP address.
    public int serverPort = 9000;             // Port for your Theta SRT stream.
    public int latency = 50;                  // Adjust based on your network.

    [Header("Video Stream Settings")]
    public int width = 1920;
    public int height = 1080;
    public int framerate = 30;
    public string pixelFormat = "bgr24";      // Raw output format: 3 bytes per pixel.

    // Internals for decoding video frames.
    private int frameSize;
    private Texture2D videoTexture;           // Will store one decoded frame.
    private Process ffmpegProcess;
    private Thread readThread;
    private byte[] frameBuffer;
    private volatile byte[] latestFrame = null;
    private readonly object frameLock = new object();
    private volatile bool isRunning = false;

    // Skybox-related fields.
    private RenderTexture skyboxRenderTexture;
    private Material skyboxMaterial;

    void Start()
    {
        // Calculate expected frame size (width * height * channels)
        frameSize = width * height * 3;  // for bgr24

        // Create the Texture2D to temporarily hold raw frame data.
        videoTexture = new Texture2D(width, height, TextureFormat.RGB24, false);

        // --- SKYBOX SETUP ---
        // Create a RenderTexture programmatically.
        skyboxRenderTexture = new RenderTexture(width, height, 0, RenderTextureFormat.Default);
        skyboxRenderTexture.Create();

        // Create a new material using Unity’s built-in panoramic skybox shader.
        // This shader is intended for equirectangular images.
        skyboxMaterial = new Material(Shader.Find("Skybox/Panoramic"));
        // Assign our RenderTexture to the material.
        skyboxMaterial.SetTexture("_MainTex", skyboxRenderTexture);
        // Optionally, set additional parameters (rotation, exposure, etc.).
        // Then assign the material as the active skybox.
        RenderSettings.skybox = skyboxMaterial;

        // --- END SKYBOX SETUP ---

        // Start the FFmpeg process to read the Theta SRT stream.
        StartFFmpeg();

        // Start the background thread to read raw frames from FFmpeg.
        isRunning = true;
        frameBuffer = new byte[frameSize];
        readThread = new Thread(ReadFrames);
        readThread.IsBackground = true;
        readThread.Start();
    }

    void StartFFmpeg()
    {
        // Build the FFmpeg command to connect to the SRT stream, scale the video,
        // set the pixel format, and output raw video frames to stdout.
        ProcessStartInfo psi = new ProcessStartInfo
        {
            FileName = ffmpegPath,
            Arguments = $"-y -fflags nobuffer -flags low_delay -strict experimental -thread_queue_size 512 -i \"srt://{serverIP}:{serverPort}?mode=caller&latency={latency}\" -vf scale={width}:{height} -pix_fmt {pixelFormat} -f rawvideo pipe:1",
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };

        ffmpegProcess = new Process { StartInfo = psi };
        ffmpegProcess.Start();

        // Start a background thread to log FFmpeg's error output.
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
            // Read exactly one frame from the FFmpeg output stream.
            while (offset < frameSize)
            {
                try
                {
                    int bytesRead = stdout.Read(frameBuffer, offset, frameSize - offset);
                    if (bytesRead == 0)
                    {
                        // If no bytes are read, the stream may have ended.
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
                // Copy the complete frame data to latestFrame.
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
        // If a new frame has been received, update our video texture.
        if (latestFrame != null)
        {
            lock (frameLock)
            {
                videoTexture.LoadRawTextureData(latestFrame);
                videoTexture.Apply();
                latestFrame = null;
            }
            // Now update the RenderTexture used by the skybox.
            // This copies the updated Texture2D (videoTexture) into the skyboxRenderTexture.
            Graphics.Blit(videoTexture, skyboxRenderTexture);
        }
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        if (readThread != null && readThread.IsAlive)
        {
            try
            {
                readThread.Abort();
            }
            catch { }
        }
        if (ffmpegProcess != null && !ffmpegProcess.HasExited)
        {
            try
            {
                ffmpegProcess.Kill();
            }
            catch { }
        }
    }
}