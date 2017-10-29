using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Willy.Core.Events;
using Willy.Core.Models;

namespace Willy.Core
{
    public class RosClient : IRosClient, IDisposable
    {
        private readonly Dictionary<string, RosServiceCall> _callQueue;
        private string _messageBuffer;

        public RosClient()
        {
            WebSocket = new ClientWebSocket();

            _callQueue = new Dictionary<string, RosServiceCall>();
            _messageBuffer = string.Empty;

            RosMessage += OnRosMessage;
        }

        public ClientWebSocket WebSocket { get; }

        public void Dispose()
        {
            WebSocket?.Dispose();
        }

        public async Task ConnectAsync()
        {
            await WebSocket.ConnectAsync(new Uri("ws://192.168.0.100:9090"), CancellationToken.None);

            // I don't care, the receive task has to run in the background anyway
#pragma warning disable 4014
            Task.Run(ReceiveTask);
#pragma warning restore 4014
        }

        public async Task<RosServiceResponse> CallServiceAsync(RosServiceCall service)
        {
            // Put the service into the queue
            var guid = Guid.NewGuid().ToString();
            _callQueue.Add(guid, service);

            // Send the request
            await WebSocket.SendAsync(service.Serialize(guid), WebSocketMessageType.Text, true, CancellationToken.None);

            // Wait for the response to be filled in
            var timeout = 0;
            while (service.Response == null)
            {
                timeout += 10;
                await Task.Delay(10);
                if (timeout >= 3000)
                    return null;
            }

            return service.Response;
        }

        public async Task SetParamValueAsync(string paramName, object param)
        {
            var service = new RosServiceCall("/rosapi/set_param")
            {
                Arguments = new Dictionary<string, object> { { "name", paramName }, { "value", "\"" + param + "\"" } }
            };
            await CallServiceAsync(service);
        }

        public async Task<object> GetParamValueAsync(string paramName)
        {
            var service = new RosServiceCall("/rosapi/get_param")
            {
                Arguments = new Dictionary<string, object> { { "name", paramName } }
            };
            var result = await CallServiceAsync(service);
            return result.TryGetValuesAsDictionary().FirstOrDefault().Value;
        }

        private async Task ReceiveTask()
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

        private void OnRosMessage(object sender, RosMessageEventArgs rosMessageEventArgs)
        {
            if (rosMessageEventArgs.Id != null && _callQueue.ContainsKey(rosMessageEventArgs.Id))
            {
                var queuedService = _callQueue[rosMessageEventArgs.Id];
                queuedService.SetResponse(queuedService.Name, rosMessageEventArgs);

                _callQueue.Remove(rosMessageEventArgs.Id);
            }
        }

        private void OnRosMessage(RosMessageEventArgs e)
        {
            var handler = RosMessage;
            handler?.Invoke(this, e);
        }

        public event EventHandler<RosMessageEventArgs> RosMessage;
    }
}
