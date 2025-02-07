#r "nuget: WebSocketSharp, 1.0.3-rc11"
#r "nuget: Newtonsoft.Json, 13.0.1"

using System;
using System.Threading;
using WebSocketSharp;
using Newtonsoft.Json;

// Global flag to quit the application
bool shouldQuit = false;
const string RELAYER_URL = "ws://132.145.67.221:9090";

void StartKeyboardThread(WebSocket ws)
{
    new Thread(() =>
    {
        Console.WriteLine("Keyboard control active:");
        Console.WriteLine("  w: forward");
        Console.WriteLine("  s: backward");
        Console.WriteLine("  a: turn left");
        Console.WriteLine("  d: turn right");
        Console.WriteLine("  x: stop");
        Console.WriteLine("  q: quit");

        while (!shouldQuit)
        {
            if (Console.KeyAvailable)
            {
                var keyInfo = Console.ReadKey(true);
                char key = char.ToLower(keyInfo.KeyChar);

                if (key == 'q')
                {
                    Console.WriteLine("Quitting...");
                    shouldQuit = true;
                    ws.Close();
                    break;
                }

                double linear_x = 0.0;
                double angular_z = 0.0;

                switch (key)
                {
                    case 'w': linear_x = 1.0; break;
                    case 's': linear_x = -1.0; break;
                    case 'a': angular_z = 1.0; break;
                    case 'd': angular_z = -1.0; break;
                    case 'x': 
                        linear_x = 0.0; 
                        angular_z = 0.0; 
                        break;
                }

                var command = new
                {
                    op = "command",
                    topic = "teleop/cmd_vel",
                    msg = new
                    {
                        linear = new { x = linear_x, y = 0.0, z = 0.0 },
                        angular = new { x = 0.0, y = 0.0, z = angular_z },
                        timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds()
                    }
                };

                string message = JsonConvert.SerializeObject(command);

                try
                {
                    ws.Send(message);
                    Console.WriteLine("Sent command: " + message);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error sending message: " + ex.Message);
                    break;
                }
            }
            Thread.Sleep(100);
        }
    })
    { IsBackground = true }.Start();
}

while (!shouldQuit)
{
    using (var ws = new WebSocket(RELAYER_URL))
    {
        ws.OnOpen += (sender, e) =>
        {
            Console.WriteLine("Connected to server.");
            StartKeyboardThread(ws);
        };

        ws.OnMessage += (sender, e) =>
        {
            Console.WriteLine("Received reply: " + e.Data);
        };

        ws.OnError += (sender, e) =>
        {
            Console.WriteLine("Error: " + e.Message);
        };

        ws.OnClose += (sender, e) =>
        {
            Console.WriteLine($"Connection closed. Code: {e.Code} Reason: {e.Reason}");
        };

        try
        {
            ws.Connect();
        }
        catch (Exception ex)
        {
            Console.WriteLine("Exception during connect: " + ex.Message);
        }

        while (ws.IsAlive && !shouldQuit)
        {
            Thread.Sleep(100);
        }
    }

    if (shouldQuit)
        break;

    Console.WriteLine("Reconnecting in 5 seconds...");
    Thread.Sleep(5000);
}
