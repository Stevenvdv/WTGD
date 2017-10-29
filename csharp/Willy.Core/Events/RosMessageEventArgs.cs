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

        public JToken Json { get; set; }

        public static RosMessageEventArgs FromJson(string json)
        {
            var res = JsonConvert.DeserializeObject<RosMessageEventArgs>(json);
            res.Json = JToken.Parse(json);
            return res;
        }

        public override string ToString()
        {
            return $"{nameof(Op)}: {Op}, {nameof(Id)}: {Id}";
        }
    }
}