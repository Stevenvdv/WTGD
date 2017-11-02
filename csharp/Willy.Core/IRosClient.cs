using System;
using System.Net.WebSockets;
using System.Threading.Tasks;
using Willy.Core.Events;
using Willy.Core.Models;

namespace Willy.Core
{
    public interface IRosClient
    {
        Task ConnectAsync(Uri uri);
        Task<RosServiceResponse> CallServiceAsync(RosServiceCall service);
        Task SetParamValueAsync(string paramName, object param);
        Task<object> GetParamValueAsync(string paramName);
        event EventHandler<RosMessageEventArgs> RosMessage;
        ClientWebSocket WebSocket { get; }
    }
}