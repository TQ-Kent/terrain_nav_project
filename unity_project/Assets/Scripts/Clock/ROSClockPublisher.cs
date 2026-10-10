using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.BuiltinInterfaces;

namespace TerrainNavigation.ClockSync
{
    public class ROSClockPublisher : MonoBehaviour
    {
        [SerializeField]
        private string m_TopicName = "/clock";
        [SerializeField]
        private float m_PublishRateSeconds = 0.02f; // 50Hz

        static ROSClockPublisher s_Active;

        private ROSConnection m_ROS;
        private float m_LastPublishTime;

        void Start()
        {
            // The TurtleBot prefab and the scene may both carry a clock publisher; keep only one.
            if (s_Active != null && s_Active != this)
            {
                Debug.LogWarning($"Duplicate ROSClockPublisher on '{name}' disabled (using '{s_Active.name}').");
                enabled = false;
                return;
            }
            s_Active = this;
            // Nav2/SLAM run on this sim clock: it must keep ticking when Unity is not the focused window.
            Application.runInBackground = true;
            QualitySettings.vSyncCount = 0;       // a blocked swap (unfocused/occluded window) would stall the sim clock
            Application.targetFrameRate = 60;
            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.RegisterPublisher<ClockMsg>(m_TopicName);
        }

        void OnDestroy()
        {
            if (s_Active == this) s_Active = null;
        }

        void Update()
        {
            if (m_ROS == null) return;
            if (Time.time - m_LastPublishTime >= m_PublishRateSeconds)
            {
                double timeSeconds = Time.timeAsDouble;
                uint sec = (uint)timeSeconds;
                uint nanosec = (uint)((timeSeconds - sec) * 1e9);

                ClockMsg clockMsg = new ClockMsg
                {
                    clock = new TimeMsg
                    {
                        sec = (int)sec,
                        nanosec = nanosec
                    }
                };

                m_ROS.Publish(m_TopicName, clockMsg);
                m_LastPublishTime = Time.time;
            }
        }
    }
}
