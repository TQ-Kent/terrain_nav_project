using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;

namespace Unity.Robotics.Core
{
    /// <summary>
    /// Converts Unity simulation time (double seconds) to ROS 2 TimeMsg (sec + nanosec).
    /// Ported from Robotics-Nav2-SLAM-Example for Unity 6 + ROS 2 Jazzy.
    /// </summary>
    public readonly struct TimeStamp
    {
        public const double k_NanosecondsInSecond = 1e9f;

        public readonly int Seconds;
        public readonly uint NanoSeconds;

        // From Unity Time.time (double seconds)
        public TimeStamp(double timeInSeconds)
        {
            var sec = Math.Floor(timeInSeconds);
            var nsec = (timeInSeconds - sec) * k_NanosecondsInSecond;
            Seconds = (int)sec;
            NanoSeconds = (uint)nsec;
        }

        // From a ROS 2 Time message
        TimeStamp(int sec, uint nsec)
        {
            Seconds = sec;
            NanoSeconds = nsec;
        }

        public static implicit operator TimeMsg(TimeStamp stamp)
        {
            return new TimeMsg(stamp.Seconds, stamp.NanoSeconds);
        }

        public static implicit operator TimeStamp(TimeMsg stamp)
        {
            return new TimeStamp(stamp.sec, stamp.nanosec);
        }
    }
}
