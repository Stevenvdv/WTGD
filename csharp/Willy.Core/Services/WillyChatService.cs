using System.Diagnostics;

namespace Willy.Core.Services
{
    public class WillyChatService : IWillyChatService
    {
        public WillyChatService()
        {
            // Create an instance of the Python Chatterbot
            Chatterbot = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };
        }

        public Process Chatterbot { get; set; }
    }

    public interface IWillyChatService
    {
    }
}