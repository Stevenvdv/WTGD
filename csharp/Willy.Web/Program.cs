using System.Net;
using Microsoft.AspNetCore;
using Microsoft.AspNetCore.Hosting;
using Microsoft.Extensions.Configuration;

namespace Willy.Web
{
    public class Program
    {
        public static void Main(string[] args)
        {
            BuildWebHost(args).Run();
        }

        public static IWebHost BuildWebHost(string[] args)
        {
            return WebHost.CreateDefaultBuilder(args)
                .ConfigureAppConfiguration((builderContext, config) => { config.AddJsonFile("appsettings.json", false, true); })
                .UseStartup<Startup>()
                .UseKestrel(options => { options.Listen(IPAddress.Any, 80); })
                .Build();
        }
    }
}