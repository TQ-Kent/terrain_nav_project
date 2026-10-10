using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

namespace TerrainNavigation.Sensors
{
    /// <summary>
    /// 2D LiDAR sensor publishing sensor_msgs/LaserScan for the 2D Baseline pipeline.
    /// Performs horizontal raycasts across configurable angular range.
    /// Ported from original LaserScanSensor with namespace updates for Unity 6.
    /// </summary>
    public class LaserScanSensor : MonoBehaviour
    {
        [Header("ROS Settings")]
        public string topic = "/scan";
        public string FrameId = "base_scan";

        [Header("Scan Parameters")]
        [FormerlySerializedAs("TimeBetweenScansSeconds")]
        public double PublishPeriodSeconds = 0.1;
        public float RangeMetersMin = 0.12f;
        public float RangeMetersMax = 3.5f;
        public float ScanAngleStartDegrees = -180f;
        public float ScanAngleEndDegrees = 180f;
        public float ScanOffsetAfterPublish = 0f;
        public int NumMeasurementsPerScan = 360;
        public float TimeBetweenMeasurementsSeconds = 0f;
        public string LayerMaskName = "";

        float m_CurrentScanAngleStart;
        float m_CurrentScanAngleEnd;
        ROSConnection m_Ros;
        double m_TimeNextScanSeconds = -1;
        int m_NumMeasurementsTaken;
        List<float> ranges = new List<float>();

        bool isScanning = false;
        double m_TimeLastScanBeganSeconds = -1;

        protected virtual void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<LaserScanMsg>(topic);

            m_CurrentScanAngleStart = ScanAngleStartDegrees;
            m_CurrentScanAngleEnd = ScanAngleEndDegrees;
            m_TimeNextScanSeconds = Clock.Now + PublishPeriodSeconds;
        }

        void BeginScan()
        {
            isScanning = true;
            m_TimeLastScanBeganSeconds = Clock.Now;
            m_TimeNextScanSeconds = m_TimeLastScanBeganSeconds + PublishPeriodSeconds;
            m_NumMeasurementsTaken = 0;
        }

        public void EndScan()
        {
            if (ranges.Count == 0)
            {
                Debug.LogWarning($"Took {m_NumMeasurementsTaken} measurements but found no valid ranges");
            }
            else if (ranges.Count != m_NumMeasurementsTaken || ranges.Count != NumMeasurementsPerScan)
            {
                Debug.LogWarning($"Expected {NumMeasurementsPerScan} measurements. Actually took {m_NumMeasurementsTaken}" +
                                 $" and recorded {ranges.Count} ranges.");
            }

            var timestamp = new TimeStamp(Clock.time);
            // Invert angle ranges: Unity (CW) -> ROS (CCW)
            var angleStartRos = -m_CurrentScanAngleStart * Mathf.Deg2Rad;
            var angleEndRos = -m_CurrentScanAngleEnd * Mathf.Deg2Rad;
            if (angleStartRos > angleEndRos)
            {
                var temp = angleEndRos;
                angleEndRos = angleStartRos;
                angleStartRos = temp;
                ranges.Reverse();
            }

            var msg = new LaserScanMsg
            {
                header = new HeaderMsg
                {
                    frame_id = FrameId,
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                range_min = RangeMetersMin,
                range_max = RangeMetersMax,
                angle_min = angleStartRos,
                angle_max = angleEndRos,
                angle_increment = (angleEndRos - angleStartRos) / NumMeasurementsPerScan,
                time_increment = TimeBetweenMeasurementsSeconds,
                scan_time = (float)PublishPeriodSeconds,
                intensities = new float[ranges.Count],
                ranges = ranges.ToArray(),
            };

            m_Ros.Publish(topic, msg);

            m_NumMeasurementsTaken = 0;
            ranges.Clear();
            isScanning = false;
            var now = (float)Clock.time;
            if (now > m_TimeNextScanSeconds)
            {
                Debug.LogWarning($"Failed to complete scan before next scheduled scan, rescheduling to now ({now:F})");
                m_TimeNextScanSeconds = now;
            }

            m_CurrentScanAngleStart += ScanOffsetAfterPublish;
            m_CurrentScanAngleEnd += ScanOffsetAfterPublish;
            if (m_CurrentScanAngleStart > 360f || m_CurrentScanAngleEnd > 360f)
            {
                m_CurrentScanAngleStart -= 360f;
                m_CurrentScanAngleEnd -= 360f;
            }
        }

        public void Update()
        {
            if (!isScanning)
            {
                if (Clock.NowTimeInSeconds < m_TimeNextScanSeconds)
                    return;
                BeginScan();
            }

            var measurementsSoFar = TimeBetweenMeasurementsSeconds == 0 ? NumMeasurementsPerScan :
                1 + Mathf.FloorToInt((float)(Clock.time - m_TimeLastScanBeganSeconds) / TimeBetweenMeasurementsSeconds);
            if (measurementsSoFar > NumMeasurementsPerScan)
                measurementsSoFar = NumMeasurementsPerScan;

            var yawBaseDegrees = transform.rotation.eulerAngles.y;
            while (m_NumMeasurementsTaken < measurementsSoFar)
            {
                var t = m_NumMeasurementsTaken / (float)NumMeasurementsPerScan;
                var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart, m_CurrentScanAngleEnd, t);
                var yawDegrees = yawBaseDegrees + yawSensorDegrees;
                var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
                var measurementStart = RangeMetersMin * directionVector + transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                // The ray starts RangeMetersMin out from the sensor, so cast only the remaining
                // distance and add the offset back to report range from the sensor origin.
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax - RangeMetersMin);

                // REP-117: a beam with no return is +inf. float.MaxValue is not treated as a clear beam
                // by slam_toolbox or the Nav2 costmaps, so free space was only mapped along beams that hit.
                if (foundValidMeasurement)
                    ranges.Add(hit.distance + RangeMetersMin);
                else
                    ranges.Add(float.PositiveInfinity);

                ++m_NumMeasurementsTaken;
            }

            if (m_NumMeasurementsTaken >= NumMeasurementsPerScan)
            {
                if (m_NumMeasurementsTaken > NumMeasurementsPerScan)
                {
                    Debug.LogError($"LaserScan has {m_NumMeasurementsTaken} measurements but expected {NumMeasurementsPerScan}");
                }
                EndScan();
            }
        }
    }
}
