using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using TerrainNavigation.Core;

namespace TerrainNavigation.Kinematics
{
    /// <summary>
    /// Differential drive robot controller subscribing to /cmd_vel (geometry_msgs/Twist).
    /// Adapted from original AGVController for Unity 6 with ArticulationBody physics.
    /// Supports both keyboard input and ROS control modes.
    /// 
    /// Robot parameters default to TurtleBot3 Waffle specifications.
    /// For terrain navigation, consider switching to WheelCollider physics
    /// when uneven terrain is introduced.
    /// </summary>
    public enum ControlMode { Keyboard, ROS }

    public class AGVController : MonoBehaviour
    {
        [Header("Wheel References")]
        public GameObject wheel1;  // Left wheel
        public GameObject wheel2;  // Right wheel

        [Header("Control Mode")]
        public ControlMode mode = ControlMode.ROS;

        [Header("Robot Parameters")]
        public float maxLinearSpeed = 2f;       // m/s
        public float maxRotationalSpeed = 1f;   // rad/s
        public float wheelRadius = 0.033f;      // meters (TB3 Waffle)
        public float trackWidth = 0.288f;       // meters (distance between wheels)
        public float forceLimit = 10f;
        public float damping = 10f;

        [Header("ROS Settings")]
        [SerializeField]
        private string m_TopicName = "cmd_vel";
        public float ROSTimeout = 0.5f;

        private ArticulationBody wA1;
        private ArticulationBody wA2;
        private ROSConnection m_ROS;
        private float lastCmdReceived = 0f;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            if (transform.root.lossyScale != Vector3.one)
            {
                Debug.LogWarning($"AGVController: robot root scale is {transform.root.lossyScale}, not (1,1,1). " +
                                 "Wheels, speeds and odometry will be wrong by that factor; reset the Scale on the robot root.");
            }
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            m_ROS = ROSConnection.GetOrCreateInstance();
            var topic = RobotNamespace.Topic(this, m_TopicName);
            m_ROS.Subscribe<TwistMsg>(topic, ReceiveROSCmd);
            Debug.Log($"AGVController: Subscribed to /{topic.TrimStart('/')}");
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
            if (!m_LoggedFirstCmd)
            {
                m_LoggedFirstCmd = true;
                Debug.Log($"AGVController: first cmd_vel received lin={rosLinear} ang={rosAngular}");
            }
        }

        bool m_LoggedFirstCmd;
        float m_NextDiagTime;

        [Tooltip("The TB3 URDF wheel inertia (~1e-5) is too small for Unity 6's solver to track a velocity drive; the wheels never spin. This overrides it.")]
        public float wheelInertia = 1e-3f;
        bool m_InertiaApplied;

        // Done in FixedUpdate (not Start) so it runs after UrdfInertial.Start() applies the URDF values.
        void ApplyWheelInertia()
        {
            if (m_InertiaApplied) return;
            m_InertiaApplied = true;
            var tensor = new Vector3(wheelInertia, wheelInertia, wheelInertia);
            wA1.inertiaTensor = tensor;
            wA2.inertiaTensor = tensor;
        }

        void LogDiagnostics()
        {
            if (Time.time < m_NextDiagTime || Time.time - lastCmdReceived > ROSTimeout) return;
            m_NextDiagTime = Time.time + 5f;
            var root = GetComponent<ArticulationBody>();
            Debug.Log($"AGV diag: mode={mode} cmdAge={Time.time - lastCmdReceived:F2}s lin={rosLinear:F2} ang={rosAngular:F2} " +
                      $"wheelTarget(L,R)=({wA1.xDrive.targetVelocity:F1},{wA2.xDrive.targetVelocity:F1}) " +
                      $"wheelJointVel(L,R)=({(wA1.jointVelocity.dofCount > 0 ? wA1.jointVelocity[0] : 0f):F2},{(wA2.jointVelocity.dofCount > 0 ? wA2.jointVelocity[0] : 0f):F2}) " +
                      $"rootVel={(root != null ? root.linearVelocity.magnitude : -1f):F3} rootImmovable={(root != null && root.immovable)} " +
                      $"sleeping(L,R)=({wA1.IsSleeping()},{wA2.IsSleeping()}) driveForce={(wA1.driveForce.dofCount > 0 ? wA1.driveForce[0] : 0f):F3} jointType={wA1.jointType} twist={wA1.twistLock} " +
                      $"driveType={wA1.xDrive.driveType} forceLimit={wA1.xDrive.forceLimit} damping={wA1.xDrive.damping} wheel1={wheel1.name} wheel2={wheel2.name}");
        }

        void FixedUpdate()
        {
            LogDiagnostics();
            ApplyWheelInertia();
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            // Unity 6 drives default to Force for prefabs serialized by older versions;
            // use a velocity drive so targetVelocity is actually tracked.
            drive.driveType = ArticulationDriveType.Velocity;
            drive.stiffness = 0f;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed = 0f;
            if (moveDirection > 0)
                inputSpeed = maxLinearSpeed;
            else if (moveDirection < 0)
                inputSpeed = -maxLinearSpeed;

            float turnDirection = Input.GetAxis("Horizontal");
            float inputRotationSpeed = 0f;
            if (turnDirection > 0)
                inputRotationSpeed = maxRotationalSpeed;
            else if (turnDirection < 0)
                inputRotationSpeed = -maxRotationalSpeed;

            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular);
        }

        /// <summary>
        /// Converts linear + angular velocity to differential wheel speeds.
        /// </summary>
        private void RobotInput(float speed, float rotSpeed)
        {
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);

            float wheel1Rotation = speed / wheelRadius;
            float wheel2Rotation = wheel1Rotation;
            // Differential drive: each wheel is offset by omega * (track / 2).
            float wheelSpeedDiff = (rotSpeed * trackWidth * 0.5f) / wheelRadius;

            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + wheelSpeedDiff) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - wheelSpeedDiff) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
            if (speed != 0f || rotSpeed != 0f)
            {
                // Articulations can fall asleep; a velocity drive target alone may not wake them.
                wA1.WakeUp();
                wA2.WakeUp();
            }
        }
    }
}
