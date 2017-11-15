using Microsoft.EntityFrameworkCore;
using Willy.Web.ControlPanel.Entities;

namespace Willy.Web.Contexts
{
    public class WillyContext : DbContext
    {
        public WillyContext(DbContextOptions<WillyContext> options) : base(options)
        {
        }

        public DbSet<Command> Commands { get; set; }
    }
}