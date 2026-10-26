using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using TerrainNavigation.Core;
using UnityEngine;

namespace TerrainNavigation.Kinematics
{
    /// <summary>
    /// Publishes the robot's transform tree to /tf for ROS 2 Jazzy.
    /// Broadcasts odom -> base_footprint -> base_link -> ... link frames.
    /// Ported from original ROSTransformTreePublisher with namespace updates.
    /// </summary>
    public class ROSTransformTreePublisher : MonoBehaviour
    {
        const string k_TfTopic = "/tf";

        [SerializeField]
        double m_PublishRateHz = 20f;
        [SerializeField]
        List<string> m_GlobalFrameIds = new List<string> { "odom" };
        // Leave "map" out when slam_toolbox owns the map->odom transform (2D SLAM mode).
        // Use { "map", "odom" } for ground-truth localization (3D pipeline).
        [SerializeField]
        GameObject m_RootGameObject;

        double m_LastPublishTimeSeconds;
        TransformTreeNode m_TransformRoot;
        ROSConnection m_ROS;

        double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
        bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

        void Start()
        {
            if (m_RootGameObject == null)
            {
                Debug.LogWarning($"No GameObject defined as {nameof(m_RootGameObject)}, using {name} as root.");
                m_RootGameObject = gameObject;
            }

            m_ROS = ROSConnection.GetOrCreateInstance();
            m_TransformRoot = new TransformTreeNode(m_RootGameObject);
            m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
            m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
        }

        static void PopulateTFList(List<TransformStampedMsg> tfList, TransformTreeNode tfNode)
        {
            foreach (var childTf in tfNode.Children)
            {
                tfList.Add(TransformTreeNode.ToTransformStamped(childTf));

                if (!childTf.IsALeafNode)
                {
                    PopulateTFList(tfList, childTf);
                }
            }
        }

        void PublishMessage()
        {
            var tfMessageList = new List<TransformStampedMsg>();

            if (m_GlobalFrameIds.Count > 0)
            {
                var tfRootToGlobal = new TransformStampedMsg(
                    new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds.Last()),
                    RobotNamespace.Frame(this, m_TransformRoot.name),
                    m_TransformRoot.Transform.To<FLU>());
                tfMessageList.Add(tfRootToGlobal);
            }
            else
            {
                Debug.LogWarning($"No {nameof(m_GlobalFrameIds)} specified, transform tree will be local coordinates only.");
            }

            // Chain global frames: map -> odom (identity transforms between them)
            for (var i = 1; i < m_GlobalFrameIds.Count; ++i)
            {
                var tfGlobalToGlobal = new TransformStampedMsg(
                    new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds[i - 1]),
                    m_GlobalFrameIds[i],
                    new TransformMsg());
                tfMessageList.Add(tfGlobalToGlobal);
            }

            var firstRobotLink = tfMessageList.Count;
            PopulateTFList(tfMessageList, m_TransformRoot);
            // Link frames are named after the GameObjects; with a namespace each robot gets its own copy.
            if (RobotNamespace.Of(this).Length > 0)
            {
                for (var i = firstRobotLink; i < tfMessageList.Count; ++i)
                {
                    tfMessageList[i].header.frame_id = RobotNamespace.Frame(this, tfMessageList[i].header.frame_id);
                    tfMessageList[i].child_frame_id = RobotNamespace.Frame(this, tfMessageList[i].child_frame_id);
                }
            }

            var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
            m_ROS.Publish(k_TfTopic, tfMessage);
            m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        }

        void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }
    }
}
