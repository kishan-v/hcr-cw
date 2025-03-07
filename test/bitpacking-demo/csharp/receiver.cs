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
                    
                    // Buffer for fixed-length data (4 + 4 + 1 + 1 + 10 = 20 bytes)
                    // int (4) + float (4) + bool (1) + char (1) + 5 shorts (10)
                    byte[] fixedBuffer = new byte[20];
                    
                    // Read the fixed-length part first
                    int bytesRead = stream.Read(fixedBuffer, 0, fixedBuffer.Length);
                    Console.WriteLine($"Read {bytesRead} bytes of fixed-length data");
                    
                    if (bytesRead == fixedBuffer.Length)
                    {
                        // Unpack the fixed-length data
                        int intValue = BitConverter.ToInt32(fixedBuffer, 0);
                        float floatValue = BitConverter.ToSingle(fixedBuffer, 4);
                        bool boolValue = BitConverter.ToBoolean(fixedBuffer, 8);
                        char charValue = (char)fixedBuffer[9];
                        
                        short[] shortArray = new short[5];
                        for (int i = 0; i < 5; i++)
                        {
                            shortArray[i] = BitConverter.ToInt16(fixedBuffer, 10 + (i * 2));
                        }
                        
                        // Read string length (2 bytes)
                        byte[] stringLenBuffer = new byte[2];
                        stream.Read(stringLenBuffer, 0, 2);
                        ushort stringLength = BitConverter.ToUInt16(stringLenBuffer, 0);
                        
                        // Read the string data
                        byte[] stringBuffer = new byte[stringLength];
                        stream.Read(stringBuffer, 0, stringLength);
                        string text = Encoding.UTF8.GetString(stringBuffer);
                        
                        // Display the unpacked values
                        Console.WriteLine("\nUnpacked data:");
                        Console.WriteLine($"Integer: {intValue}");
                        Console.WriteLine($"Float: {floatValue}");
                        Console.WriteLine($"Boolean: {boolValue}");
                        Console.WriteLine($"Character: {charValue}");
                        Console.WriteLine($"Short Array: [{string.Join(", ", shortArray)}]");
                        Console.WriteLine($"Text: \"{text}\"");
                        
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
