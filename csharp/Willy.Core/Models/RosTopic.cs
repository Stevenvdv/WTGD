using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Willy.Core.Events;

namespace Willy.Core.Models
{
    public class RosTopic : IDisposable
    {
        private readonly IRosClient _rosClient;
        private string _guid;

        public RosTopic(IRosClient rosClient, string topic, string messageType)
        {
            _rosClient = rosClient;
            _guid = Guid.NewGuid().ToString();

            Topic = topic;
            MessageType = messageType;

            Subscribe().Wait();
        }

        private void RosClientOnRosMessage(object sender, RosMessageEventArgs e)
        {
            // Only trigger a ROS message if the topics match
            if (e.Json["topic"].Value<string>() == Topic)
                OnRosMessage(e);
        }

        private async Task Subscribe()
        {
            await _rosClient.WebSocket.SendAsync(Serialize("subscribe"), WebSocketMessageType.Text, true, CancellationToken.None);
            _rosClient.RosMessage += RosClientOnRosMessage;
        }

        private async Task Unsubscribe()
        {
            if (_rosClient.WebSocket.State == WebSocketState.Open)
                await _rosClient.WebSocket.SendAsync(Serialize("unsubscribe"), WebSocketMessageType.Text, true, CancellationToken.None);
            _rosClient.RosMessage -= RosClientOnRosMessage;
        }

        public async Task Publish(object message)
        {
            if (_rosClient.WebSocket.State != WebSocketState.Open)
                return;
            
            var jObject = new JObject
            {
                ["op"] = "publish",
                ["id"] = "publish:/" + _guid,
                ["topic"] = Topic,
                ["msg"] = JToken.FromObject(message),
                ["latch"] = false,
            };

            var json = jObject.ToString();
            var bytes = Encoding.UTF8.GetBytes(json);
            await _rosClient.WebSocket.SendAsync(bytes, WebSocketMessageType.Text, true, CancellationToken.None);
        }

        private ArraySegment<byte> Serialize(string action)
        {
            // Convert the service call to JSON
            var jObject = JToken.FromObject(this, JsonSerializer.CreateDefault(new JsonSerializerSettings() {NullValueHandling = NullValueHandling.Ignore}));
            jObject["op"] = action;
            jObject["id"] = _guid;

            // Convert the JSON to a byte array
            var json = jObject.ToString();
            return Encoding.UTF8.GetBytes(json);
        }

        [JsonProperty(PropertyName = "topic")]
        public string Topic { get; }

        [JsonProperty(PropertyName = "type")]
        public string MessageType { get; }

        public void Dispose()
        {
            var task = Task.Run(async () => { await Unsubscribe(); });
            task.Wait();
        }

        private void OnRosMessage(RosMessageEventArgs e)
        {
            var handler = RosMessage;
            handler?.Invoke(this, e);
        }

        public event EventHandler<RosMessageEventArgs> RosMessage;
    }
}