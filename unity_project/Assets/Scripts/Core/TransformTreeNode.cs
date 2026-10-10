using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.Core;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace TerrainNavigation.Core
{
    /// <summary>
    /// Builds a transform tree from Unity's scene hierarchy, filtering by UrdfLink components.
    /// Each node in the tree corresponds to a URDF link that will be published as a TF frame.
    /// </summary>
    class TransformTreeNode
    {
        public readonly GameObject SceneObject;
        public readonly List<TransformTreeNode> Children;
        public Transform Transform => SceneObject.transform;
        public string name => SceneObject.name;
        public bool IsALeafNode => Children.Count == 0;

        public TransformTreeNode(GameObject sceneObject)
        {
            SceneObject = sceneObject;
            Children = new List<TransformTreeNode>();
            PopulateChildNodes(this);
        }

        public static TransformStampedMsg ToTransformStamped(TransformTreeNode node)
        {
            return node.Transform.ToROSTransformStamped(Clock.time);
        }

        static void PopulateChildNodes(TransformTreeNode tfNode)
        {
            var parentTransform = tfNode.Transform;
            for (var childIndex = 0; childIndex < parentTransform.childCount; ++childIndex)
            {
                var childTransform = parentTransform.GetChild(childIndex);
                var childGO = childTransform.gameObject;

                // Only include GameObjects with a UrdfLink component (URDF robot links)
                if (childGO.TryGetComponent(out UrdfLink _))
                {
                    var childNode = new TransformTreeNode(childGO);
                    tfNode.Children.Add(childNode);
                }
            }
        }
    }
}
