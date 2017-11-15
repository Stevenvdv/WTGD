namespace Willy.Web.ControlPanel.Entities
{
    public class Command
    {
        public int CommandId { get; set; }

        public string Name { get; set; }
        public string Description { get; set; }
        public string Content { get; set; }
    }
}