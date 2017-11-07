using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Willy.Core;
using Willy.Core.Models;

namespace Willy.ConsoleTest
{
    internal class Program
    {
        private static void Main(string[] args)
        {
            MainAsync(args).GetAwaiter().GetResult();
        }

        private static async Task MainAsync(string[] args)
        {
            var rosClient = new RosClient();
            Console.WriteLine("Connecting to ROS bridge...");
            await rosClient.ConnectAsync(new Uri("ws://10.8.0.2:9090"));
            Console.WriteLine("Connected!");

            Console.WriteLine("Subscribing to topic 'gps'...");
            var topic = new RosTopic(rosClient, "/gps", null);
            Console.ReadLine();

        }
    }
}