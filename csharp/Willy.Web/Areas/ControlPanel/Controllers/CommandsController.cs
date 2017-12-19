using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;
using Willy.Web.Contexts;
using Willy.Web.Entities;

namespace Willy.Web.Areas.ControlPanel.Controllers
{
    [Produces("application/json")]
    [Route("api/controlpanel/[controller]")]
    [Authorize(Roles = "Administrator, Manager", AuthenticationSchemes = JwtBearerDefaults.AuthenticationScheme)]
    public class CommandsController : Controller
    {
        private readonly WillyContext _context;

        public CommandsController(WillyContext context)
        {
            _context = context;
        }

        // GET: api/Commands
        [HttpGet]
        public IEnumerable<Command> GetCommands()
        {
            return _context.Commands;
        }

        // GET: api/Commands/5
        [HttpGet("{id}")]
        public async Task<IActionResult> GetCommand([FromRoute] int id)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var command = await _context.Commands.SingleOrDefaultAsync(m => m.CommandId == id);

            if (command == null)
                return NotFound();

            return Ok(command);
        }

        // PUT: api/Commands/5
        [HttpPut("{id}")]
        [Authorize(Roles = "Administrator")]
        public async Task<IActionResult> PutCommand([FromRoute] int id, [FromBody] Command command)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            if (id != command.CommandId)
                return BadRequest();

            _context.Entry(command).State = EntityState.Modified;

            try
            {
                await _context.SaveChangesAsync();
            }
            catch (DbUpdateConcurrencyException)
            {
                if (!CommandExists(id))
                    return NotFound();
                throw;
            }

            return NoContent();
        }

        // POST: api/Commands
        [HttpPost]
        [Authorize(Roles = "Administrator")]
        public async Task<IActionResult> PostCommand([FromBody] Command command)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            _context.Commands.Add(command);
            await _context.SaveChangesAsync();

            return CreatedAtAction("GetCommand", new {id = command.CommandId}, command);
        }

        // DELETE: api/Commands/5
        [HttpDelete("{id}")]
        [Authorize(Roles = "Administrator")]
        public async Task<IActionResult> DeleteCommand([FromRoute] int id)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var command = await _context.Commands.SingleOrDefaultAsync(m => m.CommandId == id);
            if (command == null)
                return NotFound();

            _context.Commands.Remove(command);
            await _context.SaveChangesAsync();

            return Ok(command);
        }

        private bool CommandExists(int id)
        {
            return _context.Commands.Any(e => e.CommandId == id);
        }
    }
}