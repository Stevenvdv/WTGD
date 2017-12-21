using System;
using System.Net;
using System.Threading.Tasks;
using Willy.Core.Services;

namespace Willy.Core.Models
{
    public class WillyChatBot : IDisposable
    {
        private readonly IWillyChatService _willyChatService;

        public WillyChatBot(IWillyChatService willyChatService)
        {
            _willyChatService = willyChatService;

            Guid = _willyChatService.GetNewBotGuid();
        }

        public string Guid { get; }

        public void Dispose()
        {
            _willyChatService.RemoveBotGuid(Guid);
        }

        public async Task<string> GetResponseAsync(string message)
        {
            var webClient = new WebClient();
            return await webClient.DownloadStringTaskAsync($"http://localhost:5000/message?guid={Guid}&msg={message}");
        }
    }
}