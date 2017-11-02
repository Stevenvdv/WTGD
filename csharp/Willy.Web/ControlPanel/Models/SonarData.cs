using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace Willy.Web.ControlPanel.Models
{
    public class SonarData
    {
        public int FrontLeftSide { get; set; }
        public int FrontLeft { get; set; }
        public int FrontCenter { get; set; }
        public int FrontRight { get; set; }
        public int FrontRightSide { get; set; }

        public int BackLeftSide { get; set; }
        public int BackLeft { get; set; }
        public int BackCenter { get; set; }
        public int BackRight { get; set; }
        public int BackRightSide { get; set; }
    }
}
