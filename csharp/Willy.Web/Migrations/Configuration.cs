using System.Linq;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Identity;
using Willy.Web.Contexts;
using Willy.Web.Entities;

namespace Willy.Web.Migrations
{
    public static class Configuration
    {
        public static async Task Seed(WillyContext context, UserManager<ApplicationUser> userManager, RoleManager<IdentityRole> roleManager)
        {
            // Create the default roles
            // Administrators have access to everything
            if (!await roleManager.RoleExistsAsync("Administrator"))
                await roleManager.CreateAsync(new IdentityRole("Administrator"));
            // Managers have access to managament functionality such as the web panel and AI management
            if (!await roleManager.RoleExistsAsync("Manager"))
                await roleManager.CreateAsync(new IdentityRole("Manager"));
            // Social interactors have access to the AI front-end
            if (!await roleManager.RoleExistsAsync("SocialInteractor"))
                await roleManager.CreateAsync(new IdentityRole("SocialInteractor"));

            // Ensure the default admin users exist and has the right roles
            var adminUser = await userManager.FindByNameAsync("Administrator");
            if (adminUser == null)
            {
                await userManager.CreateAsync(new ApplicationUser {UserName = "Administrator"}, "H00ver4l!f3");
                adminUser = await userManager.FindByNameAsync("Administrator");
            }
            await userManager.AddToRoleAsync(adminUser, "Administrator");
            await userManager.AddToRoleAsync(adminUser, "Manager");
            await userManager.AddToRoleAsync(adminUser, "SocialInteractor");

            // Create the default commands
            if (!context.Commands.Any())
            {
                context.Commands.AddRange(
                    new Command {Name = "Start up sensors", Description = "Start Willy's sensors", Content = "~/Documents/driving-willy/WTGD/bin/startup.sh"},
                    new Command {Name = "Shut down sensors", Description = "Shut down Willy's sensors", Content = "~/Documents/driving-willy/WTGD/bin/shutdown.sh"},
                    new Command {Name = "Restart system", Description = "Restart Willy's on-board computer", Content = "sudo reboot now"},
                    new Command {Name = "Shut down system", Description = "Shut down Willy's on-board computer", Content = "sudo shutdown now"},
                    new Command {Name = "Restart webpanel", Description = "Restart the webpanel, this will break the current connection", Content = "sudo service webpanel restart"}
                );
            }

            await context.SaveChangesAsync();
        }
    }
}