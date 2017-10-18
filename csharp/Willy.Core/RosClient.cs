using System;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Willy.Core.Events;
using Willy.Core.Models;

namespace Willy.Core
{
    public class RosClient : IDisposable
    {
        private string _messageBuffer;

        public RosClient()
        {
            WebSocket = new ClientWebSocket();
            ServiceQueue = new Dictionary<string, RosServiceCall>();

            _messageBuffer = string.Empty;

            RosMessage += OnRosMessage;
        }

        private void OnRosMessage(object sender, RosMessageEventArgs rosMessageEventArgs)
        {
            if (ServiceQueue.ContainsKey(rosMessageEventArgs.Id))
            {
                var queuedService = ServiceQueue[rosMessageEventArgs.Id];
                queuedService.Response = new RosServiceResponse
                {
                    Name = queuedService.Name,
                    Values = new List<object> {rosMessageEventArgs.Values}
                };
                ServiceQueue.Remove(rosMessageEventArgs.Id);
            }
        }

        public Dictionary<string, RosServiceCall> ServiceQueue { get; set; }

        public ClientWebSocket WebSocket { get; }

        public void Dispose()
        {
            WebSocket?.Dispose();
        }

        public async Task ReceiveTask()
        {
            while (WebSocket.State == WebSocketState.Open)
            {
                var bytes = new ArraySegment<byte>(new byte[4096]);
                
                var result = await WebSocket.ReceiveAsync(bytes, CancellationToken.None);
                _messageBuffer += Encoding.Default.GetString(bytes.ToArray());

                if (result.EndOfMessage)
                {
                    var args = RosMessageEventArgs.FromJson(_messageBuffer);
                    OnRosMessage(args);
                    _messageBuffer = "";
                }
            }
        }

        public async Task<RosServiceResponse> CallService(RosServiceCall service)
        {
            // Put the service into the queue
            var guid = Guid.NewGuid().ToString();
            ServiceQueue.Add(guid, service);

            // Send the request
            await WebSocket.SendAsync(service.Serialize(guid), WebSocketMessageType.Text, true, CancellationToken.None);

            // Wait for the response to be filled in
            while (service.Response == null)
                await Task.Delay(10);

            return service.Response;
        }

        public async Task ConnectAsync()
        {
            await WebSocket.ConnectAsync(new Uri("ws://192.168.0.100:9090"), CancellationToken.None);
            Task.Run(ReceiveTask);
        }

        protected virtual void OnRosMessage(RosMessageEventArgs e)
        {
            var handler = RosMessage;
            handler?.Invoke(this, e);
        }

        public event EventHandler<RosMessageEventArgs> RosMessage;
    }
}