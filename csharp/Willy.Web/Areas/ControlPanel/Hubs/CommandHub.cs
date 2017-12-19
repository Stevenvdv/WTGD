using System;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.Areas.ControlPanel.Services;

namespace Willy.Web.Areas.ControlPanel.Hubs
{
    [Route("signalr/controlpanel/command")]
    [Authorize]
    public class CommandHub : Hub
    {
        private readonly ICommandService _commandService;

        public CommandHub(ICommandService commandService)
        {
            _commandService = commandService;
        }

        public void ExecuteCommand(string command)
        {
            _commandService.Execute(command, Context.ConnectionId);
        }

        public override Task OnDisconnectedAsync(Exception exception)
        {
            _commandService.KillClientProcesses(Context.ConnectionId);
            return base.OnDisconnectedAsync(exception);
        }
    }
}