using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.Areas.ControlPanel.Services;

namespace Willy.Web.Areas.ControlPanel.Hubs
{
    [Authorize(Roles = "Administrator, Manager")]
    public class GpsHub : Hub
    {
        // ReSharper disable once PrivateFieldCanBeConvertedToLocalVariable
        private readonly IWillyRosService _willyRosService;

        public GpsHub(IWillyRosService willyRosService)
        {
            _willyRosService = willyRosService;
            _willyRosService.EnableTestData = false;
        }
    }
}
