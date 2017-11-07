using System.Collections.Generic;
using Microsoft.Extensions.Configuration;
using Renci.SshNet;

namespace Willy.Web.ControlPanel.Services
{
    public class SshService : ISshService
    {
        private readonly IConfiguration _configuration;

        public SshService(IConfiguration configuration)
        {
            _configuration = configuration;
            SshClients = new Dictionary<string, SshClient>();
        }

        public Dictionary<string, SshClient> SshClients { get; set; }

        public SshClient Connect(string connectionId, string username, string password)
        {
            if (SshClients.ContainsKey(connectionId))
                return SshClients[connectionId];

            var client = new SshClient(_configuration["SshAddress"], int.Parse(_configuration["SshPort"]), username, password);
            client.HostKeyReceived += (sender, args) => args.CanTrust = true;
            client.Connect();

            SshClients.Add(connectionId, client);
            return client;
        }

        public SshClient GetClient(string connectionId)
        {
            return !SshClients.ContainsKey(connectionId) ? null : SshClients[connectionId];
        }

        public void RemoveClient(string connectionId)
        {
            var client = GetClient(connectionId);
            if (client == null)
                return;

            SshClients.Remove(connectionId);
            client.Disconnect();
            client.Dispose();
        }
    }

    public interface ISshService
    {
        SshClient Connect(string connectionId, string username, string password);
        SshClient GetClient(string connectionId);
        void RemoveClient(string connectionId);
    }
}