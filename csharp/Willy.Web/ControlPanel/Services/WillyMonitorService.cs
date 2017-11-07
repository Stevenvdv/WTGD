using System;
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
        private readonly IRosClient _rosClient;
        
        private readonly RosTopic _gpsTopic;
        private readonly RosTopic _sonarTopic;

        public WillyMonitorService(IConfiguration configuration, IRosClient rosClient, IHubContext<GpsHub> gpsHubContext, IHubContext<SonarHub> sonarHubContext)
        {
            _rosClient = rosClient;
            _gpsHubContext = gpsHubContext;
            _sonarHubContext = sonarHubContext;
            
            Task.Run(() => _rosClient.ConnectAsync(new Uri(configuration["RosBridgeUri"]))).Wait();
            _gpsTopic = new RosTopic(_rosClient, "/gps", null);
            _gpsTopic.RosMessage += GpsTopicOnRosMessage;
            _sonarTopic = new RosTopic(_rosClient, "/sonar", null);
            _sonarTopic.RosMessage += SonarTopicOnRosMessage;
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
    }

    public interface IWillyMonitorService
    {
    }
}
