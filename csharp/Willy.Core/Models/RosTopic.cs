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
        private readonly RosClient _rosClient;
        private string _guid;

        public RosTopic(RosClient rosClient, string topic, string messageType)
        {
            _rosClient = rosClient;
            _guid = Guid.NewGuid().ToString();

            Topic = topic;
            MessageType = messageType;

            var task = Task.Run(async () => { await BridgeSubscribe(); });
            task.Wait();
        }

        private void RosClientOnRosMessage(object sender, RosMessageEventArgs rosMessageEventArgs)
        {
            Console.WriteLine(rosMessageEventArgs.Json["msg"]);
        }

        private async Task BridgeSubscribe()
        {
            await _rosClient.WebSocket.SendAsync(Serialize("subscribe"), WebSocketMessageType.Text, true, CancellationToken.None);
            _rosClient.RosMessage += RosClientOnRosMessage;
        }

        private async Task BridgeUnsubscribe()
        {
            await _rosClient.WebSocket.SendAsync(Serialize("unsubscribe"), WebSocketMessageType.Text, true, CancellationToken.None);
            _rosClient.RosMessage -= RosClientOnRosMessage;
        }

        private ArraySegment<byte> Serialize(string action)
        {
            // Convert the service call to JSON
            var jToken = JToken.FromObject(this, JsonSerializer.CreateDefault(new JsonSerializerSettings() {NullValueHandling = NullValueHandling.Ignore}));
            jToken["op"] = action;
            jToken["id"] = _guid;

            // Convert the JSON to a byte array
            var json = jToken.ToString();
            return Encoding.UTF8.GetBytes(json);
        }

        [JsonProperty(PropertyName = "topic")]
        public string Topic { get; }

        [JsonProperty(PropertyName = "type")]
        public string MessageType { get; }

        public void Dispose()
        {
            var task = Task.Run(async () => { await BridgeUnsubscribe(); });
            task.Wait();
        }
    }
}