using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using Microsoft.AspNetCore.SignalR;
using Willy.Web.Areas.ControlPanel.Hubs;

namespace Willy.Web.Areas.ControlPanel.Services
{
    public class CommandService : ICommandService
    {
        private readonly IHubContext<CommandHub> _commandHubContext;
        private readonly Dictionary<Process, string> _processes;

        public CommandService(IHubContext<CommandHub> commandHubContext)
        {
            _commandHubContext = commandHubContext;
            _processes = new Dictionary<Process, string>();
        }

        public void Execute(string command, string connectionId)
        {
            var escapedArgs = command.Replace("\"", "\\\"");

            Process process;
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                process = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "cmd.exe",
                        Arguments = $"/c \"{escapedArgs}\"",
                        RedirectStandardOutput = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    }
                };
            else
                process = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "/bin/bash",
                        Arguments = $"-c \"{escapedArgs}\"",
                        RedirectStandardOutput = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    }
                };

            _processes.Add(process, connectionId);

            var res = process.Start();
            if (res)
            {
                process.EnableRaisingEvents = true;
                process.BeginOutputReadLine();
                process.OutputDataReceived += ProcessOnOutputDataReceived;
            }
        }

        public void KillClientProcesses(string connectionId)
        {
            var processes = _processes.Where(p => p.Value == connectionId).Select(p => p.Key).ToList();
            foreach (var process in processes)
            {
                if (!process.HasExited)
                    process.Kill();
                _processes.Remove(process);
            }
        }

        private async void ProcessOnOutputDataReceived(object sender, DataReceivedEventArgs dataReceivedEventArgs)
        {
            if (!_processes.ContainsKey((Process) sender))
                return;

            var processClient = _processes[(Process) sender];
            await _commandHubContext.Clients.Client(processClient).InvokeAsync("commandOutput", dataReceivedEventArgs.Data);
        }
    }

    public interface ICommandService
    {
        void Execute(string command, string connectionId);
        void KillClientProcesses(string connectionId);
    }
}