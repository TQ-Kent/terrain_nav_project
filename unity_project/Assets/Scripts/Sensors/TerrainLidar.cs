using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;

namespace TerrainNavigation.Sensors
{
    /// <summary>
    /// 3D LiDAR sensor generating multi-ring spherical raycasts and publishing
    /// sensor_msgs/PointCloud2 to /terrain_points for the 3D Proposed pipeline.
    /// 
    /// Architecture (from architectural plan Section 5):
    /// - Horizontal FOV: 360° (-π to +π), Resolution: 0.5° (720 rays/ring)
    /// - Vertical FOV: +15° to -15° (30° total), Channels: 16/32/64 rings
    /// - Range: 0.2m - 25.0m
    /// - Publish Rate: 10 Hz
    /// - PointCloud2 fields: x(F32), y(F32), z(F32), intensity(F32) = 16 bytes/point
    /// 
    /// Uses Unity Physics.RaycastCommand (Job System) for high-throughput raycasting
    /// to avoid blocking the main thread.
    /// </summary>
    public class TerrainLidar : MonoBehaviour
    {
        [Header("ROS Settings")]
        [SerializeField] private string m_TopicName = "/terrain_points";
        [SerializeField] private string m_FrameId = "base_scan";
        [SerializeField] private float m_PublishRateHz = 10f; // 10 Hz

        [Header("Horizontal Scan")]
        [SerializeField] private float m_HorizontalFOVDegrees = 360f;
        [SerializeField] private float m_HorizontalResolutionDegrees = 0.5f;

        [Header("Vertical Scan")]
        [SerializeField] private float m_VerticalFOVUpperDegrees = 15f;
        [SerializeField] private float m_VerticalFOVLowerDegrees = -15f;
        [SerializeField] private int m_VerticalChannels = 16;

        [Header("Range")]
        [SerializeField] private float m_MinRange = 0.2f;
        [SerializeField] private float m_MaxRange = 25.0f;

        // Computed values
        private int m_HorizontalRays;
        private int m_TotalRays;
        private float m_PublishPeriodSeconds;
        private float m_LastPublishTime;

        // ROS connection
        private ROSConnection m_ROS;

        // Precomputed ray directions (in sensor-local space)
        private Vector3[] m_RayDirections;

        // PointCloud2 field definitions
        private const int POINT_STEP = 16; // x(4) + y(4) + z(4) + intensity(4)

        void Start()
        {
            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.RegisterPublisher<PointCloud2Msg>(m_TopicName);

            m_PublishPeriodSeconds = 1f / m_PublishRateHz;
            m_HorizontalRays = Mathf.CeilToInt(m_HorizontalFOVDegrees / m_HorizontalResolutionDegrees);
            m_TotalRays = m_HorizontalRays * m_VerticalChannels;

            PrecomputeRayDirections();

            Debug.Log($"TerrainLidar: {m_VerticalChannels} rings × {m_HorizontalRays} rays = {m_TotalRays} total rays @ {m_PublishRateHz} Hz");
        }

        /// <summary>
        /// Precomputes ray directions in sensor-local (Unity) coordinate space.
        /// Horizontal sweep: Unity Y-axis rotation.
        /// Vertical sweep: Unity X-axis rotation (pitch).
        /// </summary>
        void PrecomputeRayDirections()
        {
            m_RayDirections = new Vector3[m_TotalRays];

            float hStart = -m_HorizontalFOVDegrees / 2f;
            float vStep = (m_VerticalChannels > 1)
                ? (m_VerticalFOVUpperDegrees - m_VerticalFOVLowerDegrees) / (m_VerticalChannels - 1)
                : 0f;

            int index = 0;
            for (int ring = 0; ring < m_VerticalChannels; ring++)
            {
                float pitchDegrees = m_VerticalFOVUpperDegrees - ring * vStep;

                for (int h = 0; h < m_HorizontalRays; h++)
                {
                    float yawDegrees = hStart + h * m_HorizontalResolutionDegrees;
                    // Build direction: rotate forward vector by pitch (X) then yaw (Y)
                    m_RayDirections[index] = Quaternion.Euler(-pitchDegrees, yawDegrees, 0f) * Vector3.forward;
                    index++;
                }
            }
        }

        void Update()
        {
            if (Time.time - m_LastPublishTime >= m_PublishPeriodSeconds)
            {
                PerformScanAndPublish();
                m_LastPublishTime = Time.time;
            }
        }

        /// <summary>
        /// Performs raycasts using RaycastCommand batch API (Job System) and publishes PointCloud2.
        /// </summary>
        void PerformScanAndPublish()
        {
            // Allocate native arrays for the Job System
            var commands = new NativeArray<RaycastCommand>(m_TotalRays, Allocator.TempJob);
            var results = new NativeArray<RaycastHit>(m_TotalRays, Allocator.TempJob);

            // Set up raycast commands
            Vector3 origin = transform.position;
            Quaternion sensorRotation = transform.rotation;
            QueryParameters queryParams = QueryParameters.Default;

            for (int i = 0; i < m_TotalRays; i++)
            {
                Vector3 worldDir = sensorRotation * m_RayDirections[i];
                Vector3 rayOrigin = origin + worldDir * m_MinRange;
                commands[i] = new RaycastCommand(rayOrigin, worldDir, queryParams, m_MaxRange - m_MinRange);
            }

            // Schedule raycasts as a batch job
            JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, 256);
            handle.Complete();

            // Process results and build point cloud
            List<Vector3> validPoints = new List<Vector3>(m_TotalRays / 2);
            List<float> intensities = new List<float>(m_TotalRays / 2);

            for (int i = 0; i < m_TotalRays; i++)
            {
                RaycastHit hit = results[i];
                if (hit.collider != null)
                {
                    // Hit point in world space relative to sensor position
                    Vector3 hitPoint = hit.point - origin;
                    
                    float distance = hit.distance + m_MinRange;
                    if (distance >= m_MinRange && distance <= m_MaxRange)
                    {
                        // Convert from Unity (LHS: X-right, Y-up, Z-forward) 
                        // to ROS (RHS: X-forward, Y-left, Z-up)
                        float rosX =  hitPoint.z;  // Unity Z-forward -> ROS X-forward
                        float rosY = -hitPoint.x;  // Unity X-right   -> ROS Y-left (negated)
                        float rosZ =  hitPoint.y;  // Unity Y-up      -> ROS Z-up

                        validPoints.Add(new Vector3(rosX, rosY, rosZ));
                        // Intensity based on inverse distance (normalized)
                        intensities.Add(Mathf.Clamp01(1.0f - (distance / m_MaxRange)));
                    }
                }
            }

            // Dispose native arrays
            commands.Dispose();
            results.Dispose();

            // Publish PointCloud2 message
            if (validPoints.Count > 0)
            {
                PublishPointCloud2(validPoints, intensities);
            }
        }

        /// <summary>
        /// Serializes point data into sensor_msgs/PointCloud2 byte array format.
        /// Fields: x(FLOAT32,0), y(FLOAT32,4), z(FLOAT32,8), intensity(FLOAT32,12)
        /// </summary>
        void PublishPointCloud2(List<Vector3> points, List<float> intensities)
        {
            int numPoints = points.Count;
            byte[] data = new byte[numPoints * POINT_STEP];

            for (int i = 0; i < numPoints; i++)
            {
                int offset = i * POINT_STEP;
                Buffer.BlockCopy(BitConverter.GetBytes(points[i].x), 0, data, offset, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(points[i].y), 0, data, offset + 4, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(points[i].z), 0, data, offset + 8, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(intensities[i]), 0, data, offset + 12, 4);
            }

            var timestamp = new TimeStamp(Clock.time);

            var msg = new PointCloud2Msg
            {
                header = new HeaderMsg
                {
                    frame_id = m_FrameId,
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                height = 1,
                width = (uint)numPoints,
                fields = new PointFieldMsg[]
                {
                    new PointFieldMsg { name = "x", offset = 0, datatype = PointFieldMsg.FLOAT32, count = 1 },
                    new PointFieldMsg { name = "y", offset = 4, datatype = PointFieldMsg.FLOAT32, count = 1 },
                    new PointFieldMsg { name = "z", offset = 8, datatype = PointFieldMsg.FLOAT32, count = 1 },
                    new PointFieldMsg { name = "intensity", offset = 12, datatype = PointFieldMsg.FLOAT32, count = 1 },
                },
                is_bigendian = false,
                point_step = POINT_STEP,
                row_step = (uint)(POINT_STEP * numPoints),
                data = data,
                is_dense = true,
            };

            m_ROS.Publish(m_TopicName, msg);
        }
    }
}
