using Newtonsoft.Json;

namespace Willy.Core.MessageTypes
{
    public class Twist
    {
        public Twist()
        {
            Linear = new Point3D();
            Angular = new Point3D();
        }

        [JsonProperty(PropertyName = "linear")]
        public Point3D Linear { get; set; }

        [JsonProperty(PropertyName = "angular")]
        public Point3D Angular { get; set; }
    }

    public class Point3D
    {
        [JsonProperty(PropertyName = "x")]
        public double X { get; set; }

        [JsonProperty(PropertyName = "y")]
        public double Y { get; set; }

        [JsonProperty(PropertyName = "z")]
        public double Z { get; set; }
    }
}