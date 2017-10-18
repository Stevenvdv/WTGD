using System;
using System.Linq;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
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
            await rosClient.ConnectAsync();
            Console.WriteLine("Connected!");

            while (true)
            {
                Console.Write("Enter a service to call: ");
                var serviceName = Console.ReadLine();
                var service = new RosServiceCall(serviceName);
                var response = await rosClient.CallService(service);
                Console.WriteLine(response.Values.FirstOrDefault());
            }
        }
    }
}