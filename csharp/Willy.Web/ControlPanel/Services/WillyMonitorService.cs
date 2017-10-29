using System;
using System.Threading.Tasks;
using Microsoft.AspNetCore.SignalR;
using Willy.Core;
using Willy.Web.ControlPanel.Hubs;
using Willy.Web.ControlPanel.Models;

namespace Willy.Web.ControlPanel.Services
{
    public class WillyMonitorService : IWillyMonitorService
    {
        private readonly IHubContext<GpsHub> _gpsHubContext;
        private readonly IRosClient _rosClient;

        public WillyMonitorService(IRosClient rosClient, IHubContext<GpsHub> gpsHubContext)
        {
            _rosClient = rosClient;
            _gpsHubContext = gpsHubContext;

            // For now, this runs on a loop and GpsUpdate simply sends placeholder data
            // when fully implemented RosClient will send events to which this service will react, 
            // sending the data to the clients connected to the GPS hub
            Task.Run(GpsUpdateLoop);
        }

        private async Task GpsUpdateLoop()
        {
            while (true)
            {
                // Placeholder data for now
                var rand = new Random();
                var sattelites = rand.Next(7, 10);
                var lat = rand.NextDouble(52.51, 52.52);
                var longt = rand.NextDouble(6.095, 6.195);
                var gpsData = new GpsData(sattelites, lat, longt);
                await _gpsHubContext.Clients.All.InvokeAsync("gpsUpdate", gpsData);

                await Task.Delay(1000);
            }
        }
    }

    public static class RandomExtensions
    {
        public static double NextDouble(
            this Random random,
            double minValue,
            double maxValue)
        {
            return random.NextDouble() * (maxValue - minValue) + minValue;
        }
    }


    public interface IWillyMonitorService
    {
    }
}
