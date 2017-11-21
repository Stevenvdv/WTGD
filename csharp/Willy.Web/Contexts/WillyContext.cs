using Microsoft.AspNetCore.Identity.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore;
using Willy.Web.Entities;

namespace Willy.Web.Contexts
{
    public class WillyContext : IdentityDbContext<ApplicationUser>
    {
        public WillyContext(DbContextOptions<WillyContext> options) : base(options)
        {
        }

        public DbSet<Command> Commands { get; set; }
    }
}