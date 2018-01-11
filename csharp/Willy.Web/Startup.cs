using System.Text;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.HttpOverrides;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.IdentityModel.Tokens;
using Willy.Core;
using Willy.Web.Areas.ControlPanel.Hubs;
using Willy.Web.Areas.ControlPanel.Services;
using Willy.Web.Areas.Social.Hubs;
using Willy.Web.Areas.Social.Services;
using Willy.Web.Contexts;
using Willy.Web.Entities;

namespace Willy.Web
{
    public class Startup
    {
        public Startup(IConfiguration configuration)
        {
            Configuration = configuration;
        }

        public IConfiguration Configuration { get; }

        // This method gets called by the runtime. Use this method to add services to the container.
        public void ConfigureServices(IServiceCollection services)
        {
            // Configure database and authentication
            services.AddDbContext<WillyContext>(options => options.UseNpgsql(Configuration.GetConnectionString("WillyContext")));
            services.AddIdentity<ApplicationUser, IdentityRole>().AddEntityFrameworkStores<WillyContext>().AddDefaultTokenProviders();
            services.AddAuthentication(option => { option.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme; })
                .AddJwtBearer(cfg =>
                {
                    cfg.RequireHttpsMetadata = false;
                    cfg.SaveToken = true;

                    cfg.TokenValidationParameters = new TokenValidationParameters
                    {
                        ValidIssuer = Configuration["Security:Issuer"],
                        ValidAudience = Configuration["Security:Issuer"],
                        IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(Configuration["Security:SecretKey"]))
                    };

                    cfg.Events = new JwtBearerEvents
                    {
                        OnMessageReceived = context =>
                        {
                            var signalRTokenHeader = context.Request.Query["signalRTokenHeader"];
                            if (!string.IsNullOrEmpty(signalRTokenHeader))
                                context.Token = context.Request.Query["signalRTokenHeader"];
                            return Task.CompletedTask;
                        }
                    };
                });

            // Configure ASP.NET and SignalR
            services.AddCors();
            services.AddMvc();
            services.AddSignalR();

            // Register all services that will be injected later on
            // TODO: Automate
            services.AddTransient<IRosClient, RosClient>();
            services.AddSingleton<IWillyRosService, WillyRosService>();
            services.AddSingleton<IWillyChatService, WillyChatService>();
            services.AddSingleton<ICommandService, CommandService>();
        }

        // This method gets called by the runtime. Use this method to configure the HTTP request pipeline.
        public void Configure(IApplicationBuilder app, IHostingEnvironment env)
        {
            if (env.IsDevelopment())
                app.UseDeveloperExceptionPage();

            // Forward to index.html
            app.UseDefaultFiles();
            // Serve the static JavaScript files
            app.UseStaticFiles();

            app.UseForwardedHeaders(new ForwardedHeadersOptions
            {
                ForwardedHeaders = ForwardedHeaders.XForwardedFor | ForwardedHeaders.XForwardedProto
            });

            // Enable CORS from localhost on port 3000, this is used by gulp serve and thus allows the API to work when debugging
            app.UseCors(corsPolicyBuilder => corsPolicyBuilder.WithOrigins("http://localhost:3000").AllowAnyMethod().AllowAnyHeader());
            // Map MVC, actual routes are declared in the controllers
            app.UseAuthentication();
            app.UseMvc();
            // Map the SignalR hubs, routes cannot be declared in the hubs so it's done here
            app.UseSignalR(builder =>
            {
                builder.MapHub<GpsHub>("signalr/controlpanel/gps");
                builder.MapHub<SonarHub>("signalr/controlpanel/sonar");
                builder.MapHub<CommandHub>("signalr/controlpanel/command");
                builder.MapHub<RemoteControlHub>("signalr/controlpanel/remoteControl");
                builder.MapHub<ChatHub>("signalr/social/chat");
            });

            using (var serviceScope = app.ApplicationServices.GetRequiredService<IServiceScopeFactory>().CreateScope())
            {
                var context = serviceScope.ServiceProvider.GetService<WillyContext>();
                var userManager = serviceScope.ServiceProvider.GetService<UserManager<ApplicationUser>>();
                var roleManager = serviceScope.ServiceProvider.GetService<RoleManager<IdentityRole>>();
                context.Database.Migrate();
                Task.Run(() => Migrations.Configuration.Seed(context, userManager, roleManager)).Wait();
            }
        }
    }
}