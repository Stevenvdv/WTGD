using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Configuration;
using Renci.SshNet;
using Willy.Web.ControlPanel.Services;

namespace Willy.Web.ControlPanel.Hubs
{
    public class ShellHub : Hub
    {
        private readonly ISshService _sshService;

        public ShellHub(ISshService sshService)
        {
            _sshService = sshService;
        }

        public Dictionary<string, SshClient> SshClients { get; set; }

        public void ConnectToSsh(string username, string password)
        {
            var client = _sshService.Connect(Context.ConnectionId, username, password);
            client.KeepAliveInterval = TimeSpan.FromSeconds(1);
            Clients.Client(Context.ConnectionId).InvokeAsync("connectResult", client.IsConnected);
        }

        public void RunCommand(string commandText)
        {
            var client = _sshService.GetClient(Context.ConnectionId);
            if (client != null)
            {
                var commandResult = client.RunCommand(commandText);
                Clients.Client(Context.ConnectionId).InvokeAsync("commandResult", commandResult.Result);
            }
        }

        public override Task OnDisconnectedAsync(Exception exception)
        {
            _sshService.RemoveClient(Context.ConnectionId);
            return base.OnDisconnectedAsync(exception);
        }
    }
}