using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;

namespace BitPackingReceiver
{
    class Program
    {
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
                    byte[] fixedBuffer = new byte[20];
                    
                    // Read the fixed-length part first
                    int bytesRead = stream.Read(fixedBuffer, 0, fixedBuffer.Length);
                    Console.WriteLine($"Read {bytesRead} bytes of fixed-length data");
                    
                    if (bytesRead == fixedBuffer.Length)
                    {
                        // Unpack the fixed-length data
                        int width = BitConverter.ToInt32(fixedBuffer, 0);
                        int depth = BitConverter.ToInt32(fixedBuffer, 4);
                        int height = BitConverter.ToInt32(fixedBuffer, 8);
                        float stepsize = BitConverter.ToSingle(fixedBuffer, 12);
                        int timestamp = BitConverter.ToInt32(fixedBuffer, 16);

                        // Extract the packed box_vals
                        // Yes this is 4kb on the heap
                        byte[] packedBoxVals = new byte[4000];
                        stream.Read(packedBoxVals, 0, 4000);

                        // Unpack the boxvals
                        var unpackedBoxVals = new List<bool>(32000);
                        foreach (byte b in packedBoxVals)
                        {
                            for (int i = 0; i < 8; i++)
                            {
                                // Extract a bit and shift along
                                // remember this is big-endian!!
                                bool bit = (b & (1 << i)) != 0;
                                unpackedBoxVals.Add(bit);
                            }
                        }
                        
                        // Display the unpacked values
                        Console.WriteLine("\nUnpacked data:");
                        Console.WriteLine($"Width: {width}");
                        Console.WriteLine($"Depth: {depth}");
                        Console.WriteLine($"Height: {height}");
                        Console.WriteLine($"Stepsize: {stepsize}");
                        Console.WriteLine($"Timestamp: {timestamp}");
                        Console.WriteLine(string.Join(", ", unpackedBoxVals));
                        
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
