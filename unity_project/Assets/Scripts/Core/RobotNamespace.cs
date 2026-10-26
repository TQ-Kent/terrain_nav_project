using UnityEngine;

namespace TerrainNavigation.Core
{
    /// <summary>
    /// Puts one robot's ROS topics and TF frames under a namespace so several robots can share a scene.
    /// Empty = the original single-robot names ("/scan", "cmd_vel", "base_link", ...).
    /// "robot1" = "/robot1/scan", "/robot1/cmd_vel", frame "robot1/base_link". The shared frames
    /// ("map", "odom") and the "/tf" topic stay global.
    /// </summary>
    public class RobotNamespace : MonoBehaviour
    {
        public string robotNamespace = "";

        public static string Of(Component c)
        {
            var ns = c.GetComponentInParent<RobotNamespace>();
            return ns == null ? "" : ns.robotNamespace.Trim('/');
        }

        /// <summary>"/scan" -> "/robot1/scan" (leading slash kept as given).</summary>
        public static string Topic(Component c, string topic)
        {
            var ns = Of(c);
            if (ns.Length == 0) return topic;
            return topic.StartsWith("/") ? $"/{ns}{topic}" : $"{ns}/{topic}";
        }

        /// <summary>"base_link" -> "robot1/base_link".</summary>
        public static string Frame(Component c, string frame)
        {
            var ns = Of(c);
            return ns.Length == 0 ? frame : $"{ns}/{frame}";
        }
    }
}
