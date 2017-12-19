using System;
using System.Threading.Tasks;
using System.Timers;
using Microsoft.AspNetCore.SignalR;
using Willy.Core.MessageTypes;
using Willy.Core.Models;
using Willy.Web.Areas.ControlPanel.Services;

namespace Willy.Web.Areas.ControlPanel.Hubs
{
    public class RemoteControlHub : Hub
    {
        private readonly Timer _resetTimer;

        public RemoteControlHub(IWillyRosService willyRosService)
        {
            // Create the cmd_vel topic
            CmdVelTopic = new RosTopic(willyRosService.RosClient, "/cmd_vel", "geometry_msgs/Twist");

            // A timer will reset the speed to 0 if it's not reset in time
            _resetTimer = new Timer(1000) {AutoReset = true};
            _resetTimer.Elapsed += async (sender, args) => await StopMovement();
            _resetTimer.Start();
        }

        public RosTopic CmdVelTopic { get; set; }

        private async Task StopMovement()
        {
            // In case of a disconnect, ensure that Willy stops
            var message = new Twist {Linear = {X = 0}, Angular = {Z = 0}};
            // Publish the message
            await CmdVelTopic.Publish(message);
        }

        public async void Move(double x, double y)
        {
            // Reset the timer so that there are another 1000ms
            _resetTimer.Stop();
            _resetTimer.Start();

            // Translate the front-end values to something Willy understands
            var message = new Twist {Linear = {X = y}, Angular = {Z = x}};

            // Publish the message
            await CmdVelTopic.Publish(message);
        }

        public override async Task OnDisconnectedAsync(Exception exception)
        {
            // If a client disconnects stop movement right away
            await StopMovement();
            await base.OnDisconnectedAsync(exception);
        }

        protected override void Dispose(bool disposing)
        {
            // Clean up the reset timer
            _resetTimer.Stop();
            _resetTimer.Dispose();

            // Stop movement just in case
            StopMovement().Wait();

            base.Dispose(disposing);
        }
    }
}