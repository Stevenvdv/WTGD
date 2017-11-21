using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.Areas.ControlPanel.Services;

namespace Willy.Web.Areas.ControlPanel.Hubs
{
    [Authorize(Roles = "Administrator, Manager")]
    public class SonarHub : Hub
    {
        // ReSharper disable once NotAccessedField.Local
        private readonly IWillyMonitorService _willyMonitorService;

        public SonarHub(IWillyMonitorService willyMonitorService)
        {
            _willyMonitorService = willyMonitorService;
        }
    }
}
