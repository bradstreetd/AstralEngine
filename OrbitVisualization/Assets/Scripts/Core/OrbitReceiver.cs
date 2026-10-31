using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;
using UnityEngine;

namespace OrbitVisualization.Core
{
    /// <summary>
    /// Thread-safe TCP Server that listens for trajectory payloads streamed from Python.
    /// Runs a background listener thread on port 5050.
    /// </summary>
    public class OrbitReceiver : MonoBehaviour
    {
        [Header("Network Configuration")]
        [Tooltip("Port to listen on for Python connections.")]
        public int port = 5050;

        [Tooltip("Automatically start server on Awake.")]
        public bool autoStart = true;

        /// <summary>
        /// Event fired on the Unity Main Thread whenever a new orbit payload arrives.
        /// </summary>
        public static event Action<OrbitPayload> OnOrbitReceived;

        private TcpListener _tcpListener;
        private Thread _listenerThread;
        private bool _isRunning = false;

        // Thread-safe queue to safely pass received payloads from background thread to Unity main thread
        private readonly ConcurrentQueue<OrbitPayload> _payloadQueue = new ConcurrentQueue<OrbitPayload>();

        private void Awake()
        {
            if (autoStart)
            {
                StartServer();
            }
        }

        public void StartServer()
        {
            if (_isRunning) return;

            try
            {
                _isRunning = true;
                _tcpListener = new TcpListener(IPAddress.Parse("127.0.0.1"), port);
                _tcpListener.Start();

                _listenerThread = new Thread(ListenForClients)
                {
                    IsBackground = true,
                    Name = "OrbitReceiver_TCP_Thread"
                };
                _listenerThread.Start();

                Debug.Log($"<color=#38bdf8>[OrbitReceiver]</color> TCP Server started on 127.0.0.1:{port}. Waiting for Python trajectories...");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OrbitReceiver] Failed to start TCP listener on port {port}: {ex.Message}");
                _isRunning = false;
            }
        }

        private void Update()
        {
            // Process queued payloads on the Unity main thread
            while (_payloadQueue.TryDequeue(out OrbitPayload payload))
            {
                Debug.Log($"<color=#34d399>[OrbitReceiver]</color> Received new orbit: '{payload.mission_name}' with {payload.trajectory_eci?.Length ?? 0} points.");
                OnOrbitReceived?.Invoke(payload);
            }
        }

        /// <summary>
        /// Background worker thread: continuously accepts incoming connections from Python scripts.
        /// </summary>
        private void ListenForClients()
        {
            while (_isRunning)
            {
                try
                {
                    if (!_tcpListener.Pending())
                    {
                        Thread.Sleep(20);
                        continue;
                    }

                    using (TcpClient client = _tcpListener.AcceptTcpClient())
                    using (NetworkStream stream = client.GetStream())
                    {
                        // 1. Read 4-byte big-endian length prefix
                        byte[] lengthBuffer = new byte[4];
                        int bytesRead = ReadExactBytes(stream, lengthBuffer, 4);
                        if (bytesRead < 4) continue;

                        if (BitConverter.IsLittleEndian)
                        {
                            Array.Reverse(lengthBuffer);
                        }
                        int payloadLength = BitConverter.ToInt32(lengthBuffer, 0);

                        if (payloadLength <= 0 || payloadLength > 50 * 1024 * 1024) // 50MB safety limit
                        {
                            Debug.LogWarning($"[OrbitReceiver] Invalid payload length received: {payloadLength} bytes.");
                            continue;
                        }

                        // 2. Read exact payload bytes
                        byte[] payloadBuffer = new byte[payloadLength];
                        int totalPayloadRead = ReadExactBytes(stream, payloadBuffer, payloadLength);

                        if (totalPayloadRead == payloadLength)
                        {
                            string jsonString = Encoding.UTF8.GetString(payloadBuffer);
                            OrbitPayload payload = JsonConvert.DeserializeObject<OrbitPayload>(jsonString);

                            if (payload != null)
                            {
                                _payloadQueue.Enqueue(payload);
                            }
                        }
                    }
                }
                catch (SocketException)
                {
                    // Occurs when listener is stopped during shutdown; safe to break
                    break;
                }
                catch (Exception ex)
                {
                    if (_isRunning)
                    {
                        Debug.LogWarning($"[OrbitReceiver] Error processing incoming client: {ex.Message}");
                    }
                }
            }
        }

        /// <summary>
        /// Reads exactly count bytes from a stream across multiple TCP fragmentation chunks.
        /// </summary>
        private int ReadExactBytes(NetworkStream stream, byte[] buffer, int count)
        {
            int offset = 0;
            while (offset < count)
            {
                int read = stream.Read(buffer, offset, count - offset);
                if (read == 0) break; // End of stream
                offset += read;
            }
            return offset;
        }

        private void OnDestroy()
        {
            StopServer();
        }

        private void OnApplicationQuit()
        {
            StopServer();
        }

        public void StopServer()
        {
            _isRunning = false;
            try
            {
                _tcpListener?.Stop();
                _listenerThread?.Abort();
            }
            catch
            {
                // Ignored during shutdown
            }
        }
    }
}
