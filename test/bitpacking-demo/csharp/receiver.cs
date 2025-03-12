using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;

namespace BitPackingReceiver
{
    class Program
    {
        static private List<int> FindTrueIndices(byte[] data)
        {
            List<int> trueIndices = new List<int>();

            /* We are sending all the header information (depth, width, height
             * etc) even though we are apparently hardcoding all of them. We
             * can't create a copy of the whole array otherwise this would be
             * expensive, so we just skip the first 24 bytes. Yes. */
            int headerSize = 20;
            int bitIndex = 0;
            for (int i = headerSize; i < data.Length; i++)
            {
                // Loop unrolling made if faster :D
                byte b = data[i];
                if ((b & (1 << 7)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 6)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 5)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 4)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 3)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 2)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 1)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
                if ((b & (1 << 0)) != 0) trueIndices.Add(bitIndex);
                bitIndex++;
            }
            return trueIndices;
        }

        static void Main(string[] args)
        {
            // Set up the TCP listener
            TcpListener server = null;
            try
            {
                int port = 8888;
                IPAddress localAddr = IPAddress.Any;  // Listen on all interfaces
                
                // Create the TCP listener
                server = new TcpListener(localAddr, port);
                server.Start();
                
                Console.WriteLine($"Server started on port {port}, listening on all interfaces");
                Console.WriteLine("Waiting for connection...");
                
                while (true)
                {
                    // Accept client connection
                    TcpClient client = server.AcceptTcpClient();
                    Console.WriteLine("Client connected!");
                    
                    // Get stream for reading data
                    NetworkStream stream = client.GetStream();
                    
                    // Struct format on python-end is iiifL = total 20 bytes
                    byte[] fixedBuffer = new byte[4020];
                    
                    // Read the fixed-length part first
                    int bytesRead = stream.Read(fixedBuffer, 0, fixedBuffer.Length);
                    Console.WriteLine($"Read {bytesRead} bytes of fixed-length data");
                    
                    if (bytesRead == fixedBuffer.Length)
                    {
                        List<int> trueIndices = FindTrueIndices(fixedBuffer);
                        foreach (int b in trueIndices)
                        {
                            Console.WriteLine(b);
                        }

                        // Display the unpacked values
                        /* Console.WriteLine("\nUnpacked data:");
                        Console.WriteLine($"Width: {width}");
                        Console.WriteLine($"Depth: {depth}");
                        Console.WriteLine($"Height: {height}");
                        Console.WriteLine($"Stepsize: {stepsize}");
                        Console.WriteLine($"Timestamp: {timestamp}");
                        Console.WriteLine(string.Join(", ", unpackedBoxVals)); */
                        
                        // Send acknowledgment
                        string response = "Data received successfully";
                        byte[] responseBytes = Encoding.ASCII.GetBytes(response);
                        stream.Write(responseBytes, 0, responseBytes.Length);
                    }
                    
                    // Close the connection
                    client.Close();
                    Console.WriteLine("Client disconnected. Waiting for new connections...");
                }
            }
            catch (Exception e)
            {
                Console.WriteLine($"Error: {e.Message}");
            }
            finally
            {
                server?.Stop();
                Console.WriteLine("Server stopped");
            }
        }
    }
}
