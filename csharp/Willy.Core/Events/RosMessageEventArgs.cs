using System;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace Willy.Core.Events
{
    public class RosMessageEventArgs : EventArgs
    {
        [JsonProperty("op")]
        public string Op { get; set; }

        [JsonProperty("id")]
        public string Id { get; set; }

        [JsonProperty("result")]
        public bool Result { get; set; }

        [JsonProperty("values")]
        public JToken Values { get; set; }

        public static RosMessageEventArgs FromJson(string json)
        {
            return JsonConvert.DeserializeObject<RosMessageEventArgs>(json);
        }

        public override string ToString()
        {
            return $"{nameof(Op)}: {Op}, {nameof(Id)}: {Id}, {nameof(Result)}: {Result}, {nameof(Values)}: {Values}";
        }
    }
}