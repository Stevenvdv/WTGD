using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.Areas.Social.Models;
using Willy.Web.Areas.Social.Services;

namespace Willy.Web.Areas.Social.Hubs
{
    public class ChatHub : Hub
    {
        private readonly Dictionary<string, WillyChatBot> _chatBots;
        private readonly IWillyChatService _willyChatService;

        public ChatHub(IWillyChatService willyChatService)
        {
            _willyChatService = willyChatService;
            _chatBots = new Dictionary<string, WillyChatBot>();
        }

        public override Task OnConnectedAsync()
        {
            // Create a bot for the connection
            var chatBot = new WillyChatBot(_willyChatService);
            _willyChatService.ChatBots.Add(Context.ConnectionId, chatBot);

            return base.OnConnectedAsync();
        }

        public override Task OnDisconnectedAsync(Exception exception)
        {
            // Remove the bot that was created for the connection
            if (!_chatBots.ContainsKey(Context.ConnectionId))
                return base.OnDisconnectedAsync(exception);
            _chatBots[Context.ConnectionId].Dispose();
            _chatBots.Remove(Context.ConnectionId);

            return base.OnDisconnectedAsync(exception);
        }

        public async Task GetResponse(string message)
        {
            var response = await _willyChatService.ChatBots[Context.ConnectionId].GetResponseAsync(message);
            await Clients.Client(Context.ConnectionId).InvokeAsync("response", response);
        }
    }
}