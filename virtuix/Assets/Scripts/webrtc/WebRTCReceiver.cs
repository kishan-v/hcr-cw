// Author: Kishan <kishan-v@users.noreply.github.com>

using Unity.WebRTC;
using UnityEngine;
using System;
using System.Collections;
using System.Linq;
using NativeWebSocket;
using System.Collections.Concurrent;

public class WebRTCReceiver : MonoBehaviour
{
    [SerializeField] private string signallingServerUrl = "ws://130.162.176.219:8765";
    //  [SerializeField] private string signallingServerUrl = "ws://132.145.67.221:8765";  // TODO: Add signaling server URL
    [SerializeField] private LidarProcessor lidarProcessor;

    private WebSocket websocket;
    public RTCPeerConnection peerConnection;
    public System.Action<Texture> OnVideoTextureUpdated;


    // Add frame tracking variables
    private int frameCount = 0;
    private float lastFrameLogTime = 0;
    private const float FRAME_LOG_INTERVAL = 1.0f; // Log every second

    private ConcurrentQueue<string> lidarMessageQueue = new ConcurrentQueue<string>();

    [Serializable]
    public class SignalingMessage
    {
        public string type;  // message type
        public string sdp;
        public string candidate;    // For ICE candidate
        public string sdpMid;      // For ICE candidate
        public int sdpMLineIndex; // For ICE candidate
    }

    [Serializable]
    public class RestartMessage
    {
        public string type = "restart";
        public string clientType = "receiver";
        public string message = "Please restart WebRTC handshake";
    }

    [Serializable]
    public class IceCandidateMessage
    {
        public string candidate; // ICE candidate
        public string type = "candidate"; // message type i.e. "candidate"
        public string component;
        public string foundation;
        public string ip;
        public string port;
        public string priority;
        public string protocol;
        public string candidateType;
        public string relatedAddress;
        public string relatedPort;
        public string sdpMid;
        public int sdpMLineIndex;
        public string tcpType;
    }


    async void Start()
    {
        InitializeWebRTC();
        await ConnectToSignalingServer();
    }

    private void InitializeWebRTC()
    {
        StartCoroutine(WebRTC.Update());
        var configuration = new RTCConfiguration
        {
            iceServers = new[] {
                new RTCIceServer { urls = new string[] { "stun:stun.l.google.com:19302" } },
                new RTCIceServer {
                   urls = new string[] { "turn:130.162.176.219:3478?transport=udp", "turn:130.162.176.219:3478?transport=tcp" },
                   username = "username",
                   credential = "password"
                },
            },
            // iceTransportPolicy = RTCIceTransportPolicy.Relay  // Force TURN relay
        };
        peerConnection = new RTCPeerConnection(ref configuration);

        // Setup video transceiver
        var transceiverInit = new RTCRtpTransceiverInit { direction = RTCRtpTransceiverDirection.RecvOnly };
        var transceiver = peerConnection.AddTransceiver(TrackKind.Video, transceiverInit);

        // Get all available video codecs
        var codecs = RTCRtpSender.GetCapabilities(TrackKind.Video).codecs;

        // Filter codecs
        var h264Codecs = codecs.Where(codec => codec.mimeType == "video/H264");

        var error = transceiver.SetCodecPreferences(h264Codecs.ToArray());
        if (error != RTCErrorType.None)
            Debug.LogError("SetCodecPreferences failed");

        // Add connection state monitoring
        peerConnection.OnConnectionStateChange = state =>
        {
            Debug.Log($"Peer Connection State Changed: {state}");
        };

        peerConnection.OnIceConnectionChange = state =>
        {
            Debug.Log($"ICE Connection State Changed: {state}");
        };

        peerConnection.OnIceGatheringStateChange = state =>
        {
            Debug.Log($"ICE Gathering State Changed: {state}");
        };

        peerConnection.OnTrack = e =>
        {
            Debug.Log($"OnTrack event received - kind: {e.Track.Kind}");
            if (e.Track is VideoStreamTrack videoTrack)
            {
                Debug.Log("Video track received");
                videoTrack.OnVideoReceived += (Texture texture) =>
                {
                    Debug.Log("Video texture received");
                    OnVideoTextureUpdated?.Invoke(texture);
                };
            }
        };

        peerConnection.OnDataChannel = (RTCDataChannel channel) =>
        {
            Debug.Log("Data channel received: " + channel.Label);
            if (channel.Label == "lidar")
            {
                channel.OnMessage = OnLidarMessage;
                channel.OnClose = () => Debug.Log("Data Channel closed");
                channel.OnError = (error) => Debug.LogError("Data channel error " + error);
            }
        };

        peerConnection.OnIceCandidate = candidate =>
        {
            Debug.Log("OnIceCandidate event received");
            var message = new IceCandidateMessage
            {
                type = "candidate",
                candidate = candidate.Candidate,
                sdpMid = candidate.SdpMid,
                sdpMLineIndex = candidate.SdpMLineIndex.GetValueOrDefault(0),
                // sdp = candidate.Candidate,
                component = candidate.Component.ToString().ToLower(),
                foundation = candidate.Foundation,
                ip = candidate.Address.Split(':')[0],  // Split at colon and take first part - ignoring port suffix
                port = candidate.Port.ToString().ToLower(),
                priority = candidate.Priority.ToString().ToLower(),
                protocol = candidate.Protocol.ToString().ToLower(),
                candidateType = candidate.Type.ToString().ToLower(),
                relatedAddress = candidate.RelatedAddress,
                relatedPort = candidate.RelatedPort.ToString().ToLower(),
                tcpType = candidate.TcpType.ToString().ToLower()
            };
            string json = JsonUtility.ToJson(message);
            Debug.Log("Sending ICE candidate: " + json);
            websocket.SendText(json);
        };
    }

    private void OnLidarMessage(byte[] data)
    {
        try
        {
            
            string message = System.Text.Encoding.UTF8.GetString(data);
            Debug.Log("Received LiDAR data: " + message);
            if (lidarProcessor != null)
            {
                try
                {
                    lidarMessageQueue.Enqueue(message);
                }
                catch (Exception ex)
                {
                    Debug.Log("Error in adding lidar data to the queue " + ex);
                }
            }
            else
            {
                Debug.LogWarning("LidarProcessor not set. LiDAR data not processed.");
            }
        }
        catch (Exception ex)
        {
            Debug.Log("Exception in OnLidarMessage " + ex);
        }
    }

    private async System.Threading.Tasks.Task ConnectToSignalingServer()
    {
        Debug.Log($"Attempting to connect to {signallingServerUrl}");  // Add this

        websocket = new WebSocket(signallingServerUrl);

        websocket.OnMessage += HandleWebSocketMessage;
        websocket.OnError += HandleWebSocketError;
        websocket.OnClose += HandleWebSocketClose;

        websocket.OnOpen += async () =>
        {
            Debug.Log("WebSocket connection opened!");
            Debug.Log($"WebSocket State: {websocket.State}");

            // Send restart message to all connected transmitters
            RestartMessage restartMessage = new RestartMessage();
            string json = JsonUtility.ToJson(restartMessage);
            Debug.Log("Sending restart request to transmitters: " + json);
            await websocket.SendText(json);
        };

        try
        {  // Add error handling
            await websocket.Connect();
        }
        catch (Exception ex)
        {
            Debug.LogError($"Connection failed: {ex.Message}");
            return;
        }

        if (websocket.State == WebSocketState.Open)
        {
            Debug.Log("Successfully connected to websocket signaling server at " + signallingServerUrl);
        }
        else
        {
            Debug.LogWarning($"WebSocket not in Open state. Current state: {websocket.State}");
        }
    }

    private void HandleWebSocketMessage(byte[] data)
    {
        if (data == null || data.Length == 0)
        {
            Debug.LogWarning("Received empty message data");
            return;
        }
        string message = System.Text.Encoding.UTF8.GetString(data);
        Debug.Log($"Raw message received: {message}");  // Add this
        Debug.Log("Received message from signaling server: " + message);
        HandleSignalingMessage(message);
    }

    private void HandleWebSocketError(string errorMsg)
    {
        Debug.LogError($"WebSocket Error: {errorMsg}");
    }

    private void HandleWebSocketClose(WebSocketCloseCode closeCode)
    {
        Debug.Log($"WebSocket Connection Closed: {closeCode}");
    }


    /// <summary>
    /// Processes incoming signaling messages.
    /// Expects JSON messages with "type" and "sdp" fields.
    /// </summary>
    private void HandleSignalingMessage(string message)
    {
        Debug.Log("Received signaling message: " + message);
        // Parse the incoming JSON message
        SignalingMessage signalingMessage = JsonUtility.FromJson<SignalingMessage>(message);

        if (signalingMessage == null || string.IsNullOrEmpty(signalingMessage.type))
        {
            Debug.LogError("Invalid signaling message received");
            return;
        }

        // If an SDP offer is received, process it
        if (signalingMessage.type.Equals("offer", StringComparison.OrdinalIgnoreCase))
        {
            RTCSessionDescription offerDesc = new RTCSessionDescription
            {
                type = RTCSdpType.Offer,
                sdp = signalingMessage.sdp
            };

            // Process the offer (set remote description, create and send answer)
            StartCoroutine(ProcessOffer(offerDesc));
        }

        else if (signalingMessage.type.Equals("candidate", StringComparison.OrdinalIgnoreCase))
        {
            // RTCIceCandidateInit init = new RTCIceCandidateInit
            // {
            //     candidate = signalingMessage.candidate,
            //     sdpMid = signalingMessage.sdpMid,
            //     sdpMLineIndex = signalingMessage.sdpMLineIndex
            // };

            // Deserialize the ICE candidate message
            // TODO: confirm this is the correct way to deserialize the ICE candidate message RE format, spec etc.
            // In HandleSignalingMessage(...) when receiving "candidate" messages:
            IceCandidateMessage iceCandidateMessage = JsonUtility.FromJson<IceCandidateMessage>(message);

            // IMPORTANT: use the *full* candidate string, which usually starts with "candidate:"
            RTCIceCandidateInit init = new RTCIceCandidateInit
            {
                candidate = iceCandidateMessage.candidate,  // e.g. "candidate:3410918409 1 udp ..."
                sdpMid = iceCandidateMessage.sdpMid,
                sdpMLineIndex = iceCandidateMessage.sdpMLineIndex
            };
            Debug.Log("Received ICE candidate: " + init.candidate);
            var candidate = new RTCIceCandidate(init);
            peerConnection.AddIceCandidate(candidate);
        }
    }

    /// <summary>
    /// Coroutine that sets the remote description from the received offer,
    /// creates an answer, sets the local description, and sends the answer via WebSocket.
    /// </summary>
    private IEnumerator ProcessOffer(RTCSessionDescription offerDesc)
    {
        Debug.Log("Processing SDP offer...");

        // Set the remote description using the received offer
        var opRemote = peerConnection.SetRemoteDescription(ref offerDesc);
        yield return opRemote;
        if (opRemote.IsError)
        {
            Debug.LogError("Error setting remote description: " + opRemote.Error.message);
            yield break;
        }

        // Create an answer
        var opAnswer = peerConnection.CreateAnswer();
        yield return opAnswer;
        if (opAnswer.IsError)
        {
            Debug.LogError("Error creating answer: " + opAnswer.Error.message);
            yield break;
        }
        RTCSessionDescription answerDesc = opAnswer.Desc;

        // Set the local description with the created answer
        var opLocal = peerConnection.SetLocalDescription(ref answerDesc);
        yield return opLocal;
        if (opLocal.IsError)
        {
            Debug.LogError("Error setting local description: " + opLocal.Error.message);
            yield break;
        }

        // Prepare and send the answer back via the signaling server
        SignalingMessage answerMessage = new SignalingMessage
        {
            type = "answer",
            sdp = answerDesc.sdp
        };
        string answerJson = JsonUtility.ToJson(answerMessage);
        Debug.Log("Sending answer: " + answerJson);
        websocket.SendText(answerJson);
    }

    private async void OnDestroy()
    {
        if (websocket != null && websocket.State == WebSocketState.Open)
            await websocket.Close();
        peerConnection?.Close();
    }

    private void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        if (websocket != null)
        {
            websocket.DispatchMessageQueue();
        }
        else
        {
            Debug.LogWarning("WebSocket is null");
        }
#endif

        int messagesToProcess = 5; // adjust this number as needed
        while (messagesToProcess > 0 && lidarMessageQueue.TryDequeue(out string lidarString))
        {
            if (lidarProcessor != null)
            {
                try
                {
                    lidarProcessor.ProcessLidarData(lidarString);
                }
                catch (Exception ex)
                {
                    Debug.LogError("Error in processing LiDAR Data " + ex);
                }
            }
            else
            {
                Debug.LogWarning("LidarProcessor not set. LiDAR data not processed.");
            }
            messagesToProcess--;
        }
    }
}