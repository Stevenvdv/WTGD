using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace Willy.Core.Models
{
    public class RosServiceResponse
    {
        public string Name { get; set; }
        public JToken Values { get; set; }

        public Dictionary<string, object> TryGetValuesAsDictionary()
        {
            try
            {
                var res = Values.ToObject<Dictionary<string, object>>();
                return res;
            }
            catch (Exception)
            {
                return null;
            }
        }
    }
}