using System;
using System.Collections.Generic;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Willy.Core.Events;

namespace Willy.Core.Models
{
    public class RosServiceCall
    {
        public RosServiceCall(string name)
        {
            Name = name;
        }

        [JsonProperty(PropertyName = "service")]
        public string Name { get; set; }

        [JsonProperty(PropertyName = "args")]
        public Dictionary<string, object> Arguments { get; set; }

        [JsonIgnore]
        public RosServiceResponse Response { get; set; }

        public ArraySegment<byte> Serialize(string id)
        {
            // Convert the service call to JSON
            var jToken = JToken.FromObject(this);
            jToken["op"] = "call_service";
            jToken["id"] = id;

            // Convert the JSON to a byte array
            var json = jToken.ToString();
            return Encoding.UTF8.GetBytes(json);
        }

        public void SetResponse(string name, RosMessageEventArgs objects)
        {
            Response = new RosServiceResponse {Name = name, Values = objects.Json["values"]};
        }
    }
}