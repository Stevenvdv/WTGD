using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.ControlPanel.Services;

namespace Willy.Web.ControlPanel.Hubs
{
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