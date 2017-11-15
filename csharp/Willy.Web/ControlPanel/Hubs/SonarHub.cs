using Microsoft.AspNetCore.SignalR;
using Willy.Web.ControlPanel.Services;

namespace Willy.Web.ControlPanel.Hubs
{
    public class SonarHub : Hub
    {
        private readonly IWillyMonitorService _willyMonitorService;

        public SonarHub(IWillyMonitorService willyMonitorService)
        {
            _willyMonitorService = willyMonitorService;
        }
    }
}
