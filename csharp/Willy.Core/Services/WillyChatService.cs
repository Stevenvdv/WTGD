using System;
using System.Diagnostics;
using System.IO;
using System.Net;

namespace Willy.Core.Services
{
    public class WillyChatService : IWillyChatService, IDisposable
    {
        private readonly string _chatBotLocation;
        private readonly Process _server;

        public WillyChatService()
        {
            _chatBotLocation = "C:\\willy-chatterbot\\";
            // Create an instance of the Python Chatterbot
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
        }

        public bool IsTrained => File.Exists(_chatBotLocation + "database.sqlite3") &&
                                 new FileInfo(_chatBotLocation + "database.sqlite3").Length > 200;

        public void Dispose()
        {
            _server?.Kill();
            _server?.Dispose();
        }

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
    }

    public interface IWillyChatService
    {
        bool IsTrained { get; }
        void TrainBots();
        string GetNewBotGuid();
        void RemoveBotGuid(string guid);
        void RemoveAllBots();
    }
}