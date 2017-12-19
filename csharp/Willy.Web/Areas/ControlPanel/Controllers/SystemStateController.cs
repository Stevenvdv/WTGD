using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Willy.Web.Areas.ControlPanel.Models;

namespace Willy.Web.Areas.ControlPanel.Controllers
{
    [Produces("application/json")]
    [Route("api/controlpanel/[controller]")]
    [Authorize(Roles = "Administrator, Manager", AuthenticationSchemes = JwtBearerDefaults.AuthenticationScheme)]
    public class SystemStateController : Controller
    {
        [HttpGet]
        public SystemState GetSystemState()
        {
            var systemState = new SystemState();
            systemState.Build();
            return systemState;
        }
    }
}