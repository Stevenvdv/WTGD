using System;
using System.Net.WebSockets;
using System.Threading.Tasks;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Configuration;
using Newtonsoft.Json.Linq;
using Willy.Core;
using Willy.Core.Events;
using Willy.Core.Models;
using Willy.Web.ControlPanel.Hubs;
using Willy.Web.ControlPanel.Models;

namespace Willy.Web.ControlPanel.Services
{
    public class WillyMonitorService : IWillyMonitorService
    {
        private readonly IHubContext<GpsHub> _gpsHubContext;
        private readonly IHubContext<SonarHub> _sonarHubContext;
        private readonly IConfiguration _configuration;
        private readonly IRosClient _rosClient;
        
        private RosTopic _gpsTopic;
        private RosTopic _sonarTopic;
        private bool _running;

        public WillyMonitorService(IConfiguration configuration, IRosClient rosClient, IHubContext<GpsHub> gpsHubContext, IHubContext<SonarHub> sonarHubContext)
        {
            _configuration = configuration;
            _rosClient = rosClient;
            _gpsHubContext = gpsHubContext;
            _sonarHubContext = sonarHubContext;
            _running = true;

            // A continuous task keeps the ROS connection in a good state
            Task.Run(ClientStateTask);
        }

        private async Task ClientStateTask()
        {
            while (_running)
            {
                if (_rosClient.WebSocket.State == WebSocketState.Open)
                    continue;
                
                // Remove the old topics if they exist 
                _gpsTopic?.Dispose();
                _sonarTopic?.Dispose();

                // Create a new connection and topics
                try
                {
                    await _rosClient.ConnectAsync(new Uri(_configuration["RosBridgeUri"]));
                }
                catch (Exception e)
                {
                    Console.WriteLine("ROS connection failed:");
                    Console.WriteLine(e);

                    await Task.Delay(1000);
                    continue;
                }
                
                _gpsTopic = new RosTopic(_rosClient, "/gps", null);
                _gpsTopic.RosMessage += GpsTopicOnRosMessage;
                _sonarTopic = new RosTopic(_rosClient, "/sonar", null);
                _sonarTopic.RosMessage += SonarTopicOnRosMessage;

                await Task.Delay(1000);
            }
        }

        private async void GpsTopicOnRosMessage(object sender, RosMessageEventArgs e)
        {
            // The GPS data is a raw string, parse it into normal data
            var data = e.Json["msg"]["data"].Value<string>();
            var values = data.Split(',');
            var gpsData = new GpsData(int.Parse(values[0]), double.Parse(values[1]), double.Parse(values[2]));

            // Send the GPS data to all clients connected to the GPS hub
            await _gpsHubContext.Clients.All.InvokeAsync("gpsUpdate", gpsData);
        }

        private async void SonarTopicOnRosMessage(object sender, RosMessageEventArgs e)
        {
            var echoes = e.Json["msg"]["echoes"];
            var sonarData = new SonarData
            {
                FrontLeftSide = echoes[9].Value<int>(),
                FrontLeft = echoes[8].Value<int>(),
                FrontCenter = echoes[7].Value<int>(),
                FrontRight = echoes[6].Value<int>(),
                FrontRightSide = echoes[5].Value<int>(),
                BackLeftSide = echoes[4].Value<int>(),
                BackLeft = echoes[3].Value<int>(),
                BackCenter = echoes[2].Value<int>(),
                BackRight = echoes[1].Value<int>(),
                BackRightSide = echoes[0].Value<int>()
            };

            // Send the sonar data to all clients connected to the sonar hub
            await _sonarHubContext.Clients.All.InvokeAsync("sonarUpdate", sonarData);
        }

        public void Dispose()
        {
            _running = false;
            _gpsTopic?.Dispose();
            _sonarTopic?.Dispose();
        }
    }

    public interface IWillyMonitorService : IDisposable
    {
    }
}
