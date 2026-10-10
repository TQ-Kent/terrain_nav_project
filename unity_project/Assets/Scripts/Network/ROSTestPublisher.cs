using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace TerrainNavigation.Network
{
    public class ROSTestPublisher : MonoBehaviour
    {
        [SerializeField]
        private string m_TopicName = "/unity_status";
        [SerializeField]
        private float m_PublishRateSeconds = 1.0f; // 1Hz

        private ROSConnection m_ROS;
        private float m_LastPublishTime;

        void Start()
        {
            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.RegisterPublisher<StringMsg>(m_TopicName);
            Debug.Log($"ROS Test Publisher started on topic: {m_TopicName}");
        }

        void Update()
        {
            if (Time.time - m_LastPublishTime >= m_PublishRateSeconds)
            {
                StringMsg msg = new StringMsg
                {
                    data = $"Unity 6 connected to ROS 2 Jazzy! Sim Time: {Time.time:F2}s"
                };

                m_ROS.Publish(m_TopicName, msg);
                m_LastPublishTime = Time.time;
            }
        }
    }
}
