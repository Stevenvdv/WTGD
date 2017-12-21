using System;
using System.Threading;
using System.Threading.Tasks;
using Willy.Core;
using Willy.Core.MessageTypes;
using Willy.Core.Models;
using Willy.Core.Services;

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
            //            var rosClient = new RosClient();
            //            Console.WriteLine("Connecting to ROS bridge...");
            //            await rosClient.ConnectAsync(new Uri("ws://192.168.0.100:10500"));
            //            Console.WriteLine("Connected!");
            //
            //            Console.WriteLine("Initializing cmd_vel topic");
            //            var topic = new RosTopic(rosClient, "/cmd_vel", "geometry_msgs/Twist");
            //            var message = new Twist();
            //            await topic.Publish(message);
            //            Console.ReadLine();

            var service = new WillyChatService();
            var chatBot = new WillyChatBot(service);

            Console.WriteLine("Start writing your messages to see the bot's response");
            while (true)
            {
                var message = Console.ReadLine();
                if (!string.IsNullOrWhiteSpace(message))
                    Console.WriteLine(await chatBot.GetResponseAsync(message));
            }
        }
    }
}