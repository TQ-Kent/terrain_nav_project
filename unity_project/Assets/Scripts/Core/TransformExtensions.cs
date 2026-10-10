using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace TerrainNavigation.Core
{
    /// <summary>
    /// Extension methods for converting Unity Transform to ROS TransformMsg/TransformStampedMsg.
    /// Handles Unity (LHS) to ROS (FLU/RHS) coordinate frame conversion.
    /// </summary>
    public static class TransformExtensions
    {
        public static TransformMsg ToROSTransform(this Transform tfUnity)
        {
            return new TransformMsg(
                // Use localPosition/localRotation for parent-relative transforms
                tfUnity.localPosition.To<FLU>(),
                tfUnity.localRotation.To<FLU>());
        }

        public static TransformStampedMsg ToROSTransformStamped(this Transform tfUnity, double timeStamp)
        {
            return new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(timeStamp), tfUnity.parent.gameObject.name),
                tfUnity.gameObject.name,
                tfUnity.ToROSTransform());
        }
    }
}
