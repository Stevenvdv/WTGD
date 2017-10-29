namespace Willy.Web.ControlPanel.Models
{
    public class GpsData
    {
        public GpsData(int satellites, double latitude, double longitude)
        {
            Satellites = satellites;
            Latitude = latitude;
            Longitude = longitude;
        }

        public int Satellites { get; set; }
        public double Latitude { get; set; }
        public double Longitude { get; set; }
    }
}
