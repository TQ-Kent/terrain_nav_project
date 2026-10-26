"""One CBBA agent per robot: agree on tasks with the other robots, then drive the agreed path with Nav2.

Topics (global): /cbba/tasks (in, latched), /cbba/messages (in and out), /cbba/done (out).
Nav2 action: <namespace>/navigate_to_pose.
"""
import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from terrain_multirobot_msgs.msg import CbbaMsg, TaskDone, TaskList
from .allocation import RobotAllocator, from_fields


class CbbaAgent(Node):
    def __init__(self):
        super().__init__('cbba_agent')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 2)
        self.declare_parameter('namespace', '')          # Nav2 namespace, e.g. 'robot0'
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')  # already prefixed, e.g. 'robot0/base_link'
        self.declare_parameter('speed', 0.8)              # m/s used for the arrival-time estimate
        self.declare_parameter('max_bundle', 2)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('quiet_steps', 6)
        self.declare_parameter('task_radius', 0.3)        # accepted distance to a task point

        p = self.get_parameter
        self.rid = p('robot_id').value
        self.n = p('num_robots').value
        self.ns = p('namespace').value
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.task_radius = p('task_radius').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskList, '/cbba/tasks', self.on_tasks, latched)
        self.create_subscription(CbbaMsg, '/cbba/messages', self.on_message, 20)
        self.msg_pub = self.create_publisher(CbbaMsg, '/cbba/messages', 20)
        self.done_pub = self.create_publisher(TaskDone, '/cbba/done', 20)
        action = (f'/{self.ns}/navigate_to_pose' if self.ns else 'navigate_to_pose')
        self.nav = ActionClient(self, NavigateToPose, action)

        self.alloc = None
        self.inbox = []
        self.phase = 'waiting'   # waiting -> bidding -> driving -> done
        self.queue = []
        self.current = None
        self.tasks = []
        self.timer = self.create_timer(1.0 / p('rate_hz').value, self.tick)

    # --- inputs
    def on_tasks(self, msg):
        if self.alloc is not None:
            return
        self.tasks = [(t.x, t.y, t.value, t.discount) for t in sorted(msg.tasks, key=lambda t: t.id)]
        self.get_logger().info(f'received {len(self.tasks)} tasks')

    def on_message(self, msg):
        if msg.sender_id != self.rid:
            self.inbox.append(from_fields(msg.sender_id, msg.winning_bid, msg.winning_agent, msg.timestamps))

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    # --- loop
    def tick(self):
        if self.phase == 'waiting':
            if not self.tasks:
                return
            pos = self.pose()
            if pos is None:
                return
            p = self.get_parameter
            self.alloc = RobotAllocator(self.rid, pos, self.tasks, self.n, speed=p('speed').value,
                                        max_bundle=p('max_bundle').value, quiet_steps=p('quiet_steps').value)
            self.phase = 'bidding'
            self.get_logger().info(f'robot {self.rid} starts bidding from {pos[0]:.2f}, {pos[1]:.2f}')
        if self.phase == 'bidding':
            inbox, self.inbox = self.inbox, []
            out = self.alloc.step(inbox)
            m = CbbaMsg(sender_id=out.sender_id, winning_bid=out.winning_bid_list,
                        winning_agent=out.winning_agent_list, timestamps=out.timestamp_list)
            self.msg_pub.publish(m)
            if self.alloc.converged():
                self.queue = self.alloc.path
                self.get_logger().info(f'robot {self.rid} agreed, path {self.queue}')
                self.phase = 'driving'
            return
        if self.phase == 'driving':
            # keep broadcasting the final state so slower robots still converge
            out = self.alloc.step([])
            self.msg_pub.publish(CbbaMsg(sender_id=out.sender_id, winning_bid=out.winning_bid_list,
                                         winning_agent=out.winning_agent_list, timestamps=out.timestamp_list))
            if self.current is None:
                self.drive_next()

    def drive_next(self):
        if not self.queue:
            self.phase = 'done'
            self.get_logger().info(f'robot {self.rid} finished all tasks')
            return
        j = self.queue[0]
        x, y, _, _ = self.tasks[j]
        if not self.nav.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.w = 1.0
        self.current = j
        self.get_logger().info(f'robot {self.rid} -> task {j} at ({x:.2f}, {y:.2f})')
        self.nav.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'task {self.current} rejected by Nav2')
            self.finish_task(False)
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.finish_task(future.result().status == 4)  # STATUS_SUCCEEDED

    def finish_task(self, ok):
        j = self.current
        self.current = None
        self.queue.pop(0)
        if ok:
            self.done_pub.publish(TaskDone(task_id=j, robot_id=self.rid))
        else:
            self.get_logger().warn(f'robot {self.rid} failed task {j}; skipping it')


def main(args=None):
    rclpy.init(args=args)
    node = CbbaAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
