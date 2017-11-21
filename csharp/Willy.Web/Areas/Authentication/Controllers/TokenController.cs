using System;
using System.Collections.Generic;
using System.IdentityModel.Tokens.Jwt;
using System.Security.Claims;
using System.Text;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.IdentityModel.Tokens;
using Willy.Web.Areas.Authentication.Models;
using Willy.Web.Entities;

namespace Willy.Web.Areas.Authentication.Controllers
{
    [Produces("application/json")]
    [Route("api/authentication/token")]
    public class TokenController : Controller
    {
        private static readonly TimeSpan ExpireSpan = TimeSpan.FromHours(1);
        private readonly IConfiguration _configuration;
        private readonly RoleManager<IdentityRole> _roleManager;
        private readonly UserManager<ApplicationUser> _userManager;

        public TokenController(IConfiguration configuration, UserManager<ApplicationUser> userManager, RoleManager<IdentityRole> roleManager)
        {
            _configuration = configuration;
            _userManager = userManager;
            _roleManager = roleManager;
        }

        [HttpPost("create")]
        public async Task<IActionResult> Create([FromBody] LoginModel loginModel)
        {
            var user = await _userManager.FindByNameAsync(loginModel.UserName);
            if (user == null)
                return BadRequest();
            if (!await _userManager.CheckPasswordAsync(user, loginModel.Password))
                return BadRequest();

            var claims = new List<Claim>
            {
                new Claim(JwtRegisteredClaimNames.Sub, user.UserName),
                new Claim(JwtRegisteredClaimNames.Jti, Guid.NewGuid().ToString())
            };

            foreach (var userRole in await _userManager.GetRolesAsync(user))
                claims.Add(new Claim(ClaimTypes.Role, userRole));

            var key = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(_configuration["Security:SecretKey"]));
            var creds = new SigningCredentials(key, SecurityAlgorithms.HmacSha256);

            var token = new JwtSecurityToken(_configuration["Security:Issuer"],
                _configuration["Security:Issuer"],
                claims,
                expires: DateTime.Now.AddMinutes(30),
                signingCredentials: creds);

            return Ok(new { token = new JwtSecurityTokenHandler().WriteToken(token) });
        }


    }
}