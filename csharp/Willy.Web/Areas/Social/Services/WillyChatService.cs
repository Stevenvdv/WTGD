using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Threading.Tasks;
using Microsoft.Extensions.Configuration;
using Willy.Web.Areas.Social.Models;

namespace Willy.Web.Areas.Social.Services
{
    public class WillyChatService : IWillyChatService, IDisposable
    {
        private readonly string _chatBotLocation;
        private bool _mustRun;
        private Process _server;

        public WillyChatService(IConfiguration configuration)
        {
            _chatBotLocation = configuration["ChatBotLocation"];
            ChatBots = new Dictionary<string, WillyChatBot>();

            // Start a task that makes sure the Chatterbot server is running
            Task.Run(ChatterbotMonitorTask);
        }

        public void Dispose()
        {
            _mustRun = false;
            _server?.Kill();
            _server?.Dispose();
        }

        public Dictionary<string, WillyChatBot> ChatBots { get; set; }

        public bool IsTrained => File.Exists(_chatBotLocation + "database.sqlite3") &&
                                 new FileInfo(_chatBotLocation + "database.sqlite3").Length > 200;

        public void TrainBots()
        {
            if (IsTrained)
                return;

            var webClient = new WebClient();
            webClient.DownloadString("http://localhost:5000/train");
        }

        public string GetNewBotGuid()
        {
            var webClient = new WebClient();
            return webClient.DownloadString("http://localhost:5000/create");
        }

        public void RemoveBotGuid(string guid)
        {
            var webClient = new WebClient();
            webClient.DownloadString($"http://localhost:5000/remove?guid={guid}");
        }

        public void RemoveAllBots()
        {
            var webClient = new WebClient();
            webClient.DownloadString("http://localhost:5000/reset");
        }

        public bool IsRunning()
        {
            var webClient = new WebClient();
            try
            {
                webClient.DownloadString("http://localhost:5000");
                return true;
            }
            catch (Exception)
            {
                // Not a great way of doing this
                return false;
            }
        }

        public event EventHandler ChatServerStarted;

        private async Task ChatterbotMonitorTask()
        {
            _mustRun = true;
            while (_mustRun)
            {
                // Try to access the Chatterbot
                if (IsRunning())
                {
                    await Task.Delay(5000);
                    continue;
                }

                // Start the Python script running the Chatterbot server
                // Create an instance of the Python Chatterbot
                _server?.Kill();
                _server?.Dispose();
                _server = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = _chatBotLocation + "Willy.Chatbot.py",
                        WorkingDirectory = _chatBotLocation,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    }
                };
                _server.Start();
                OnChatServerStarted();
                await Task.Delay(5000);
            }
        }

        protected virtual void OnChatServerStarted()
        {
            ChatServerStarted?.Invoke(this, EventArgs.Empty);
        }
    }

    public interface IWillyChatService
    {
        Dictionary<string, WillyChatBot> ChatBots { get; set; }
        bool IsTrained { get; }
        void Dispose();
        void TrainBots();
        string GetNewBotGuid();
        void RemoveBotGuid(string guid);
        void RemoveAllBots();
        bool IsRunning();
        event EventHandler ChatServerStarted;
    }
}