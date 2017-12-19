using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;

namespace Willy.Web.Areas.ControlPanel.Models
{
    public class SystemState
    {
        public List<DriveState> Drives { get; set; }
        public List<ProcessState> Processes { get; set; }

        public void Build()
        {
            // Hard drive information
            Drives = new List<DriveState>();
            var drives = DriveInfo.GetDrives();
            foreach (var driveInfo in drives)
                Drives.Add(new DriveState(driveInfo));

            // Process information
            Processes = new List<ProcessState>();
            var processes = Process.GetProcesses();
            foreach (var process in processes)
                Processes.Add(new ProcessState(process));
        }
    }

    public class DriveState
    {
        public DriveState(DriveInfo driveInfo)
        {
            Name = driveInfo.Name;
            DriveType = driveInfo.DriveType;

            // Some devices such as disk drives don't have this information if they aren't ready (no disk inserted)
            if (driveInfo.IsReady)
            {
                DriveFormat = driveInfo.DriveFormat;
                TotalFreeSpace = driveInfo.TotalFreeSpace;
                AvailableFreeSpace = driveInfo.AvailableFreeSpace;
                TotalSize = driveInfo.TotalSize;
            }
        }

        public long TotalSize { get; set; }
        public long AvailableFreeSpace { get; set; }
        public long TotalFreeSpace { get; set; }
        public string DriveFormat { get; set; }
        public DriveType DriveType { get; }
        public string Name { get; set; }
    }

    public class ProcessState
    {
        public ProcessState(Process process)
        {
            Name = process.ProcessName;
            MemoryUsage = process.WorkingSet64;
            try
            {
                CpuUsage = process.TotalProcessorTime;
            }
            catch (Win32Exception e)
            {
                // ignored, happens on certain system processes
            }
        }

        public TimeSpan CpuUsage { get; set; }
        public long MemoryUsage { get; set; }
        public string Name { get; set; }
    }
}