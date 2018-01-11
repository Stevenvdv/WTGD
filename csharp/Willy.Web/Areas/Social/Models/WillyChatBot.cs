using System;
using System.Net;
using System.Threading.Tasks;
using Willy.Web.Areas.Social.Services;

namespace Willy.Web.Areas.Social.Models
{
    public class WillyChatBot : IDisposable
    {
        private readonly IWillyChatService _willyChatService;

        public WillyChatBot(IWillyChatService willyChatService)
        {
            _willyChatService = willyChatService;

            // If already running, create a GUID right away
            if (_willyChatService.IsRunning())
                Guid = _willyChatService.GetNewBotGuid();
            // If not, wait and create it when the server started
            _willyChatService.ChatServerStarted += OnServerStarted;
        }

        public string Guid { get; set; }

        public void Dispose()
        {
            _willyChatService.RemoveBotGuid(Guid);
        }

        private void OnServerStarted(object sender, EventArgs args)
        {
            Guid = _willyChatService.GetNewBotGuid();
        }

        public async Task<string> GetResponseAsync(string message)
        {
            if (Guid == null)
                return "Wacht nog even, ik ben nog niet klaar.";

            var webClient = new WebClient();
            return await webClient.DownloadStringTaskAsync($"http://localhost:5000/message?guid={Guid}&msg={message}");
        }
    }
}