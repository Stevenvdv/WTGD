using Microsoft.AspNetCore.SignalR;
using Willy.Web.ControlPanel.Services;

namespace Willy.Web.ControlPanel.Hubs
{
    public class GpsHub : Hub
    {
        private readonly IWillyMonitorService _willyMonitorService;

        public GpsHub(IWillyMonitorService willyMonitorService)
        {
            _willyMonitorService = willyMonitorService;
            _willyMonitorService.EnableTestData = false;
        }
    }
}
