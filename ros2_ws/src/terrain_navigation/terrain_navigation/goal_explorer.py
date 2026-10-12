"""Drive to a known goal location through an unknown map using goal-biased frontier exploration.

The goal coordinates (map frame, metres) are given up front; the map is not. While the goal
is not yet reachable through known free space, the node sends Nav2 the best frontier (see
frontier_logic). Once the goal is reachable it sends the goal itself.

    ros2 run terrain_navigation goal_explorer --ros-args -p goal_x:=10.0 -p goal_y:=-10.0
"""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros

from terrain_navigation import frontier_logic as fl


class GoalExplorer(Node):

    def __init__(self):
        super().__init__('goal_explorer')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_weight', 1.0)       # pull toward the goal vs. path cost
        self.declare_parameter('clearance', 0.05)        # extra m from costmap obstacles (inflation already covers the robot)
        self.declare_parameter('min_frontier_cells', 3)
        self.declare_parameter('switch_margin', 1.0)     # m of score gain needed to change target
        self.declare_parameter('blacklist_seconds', 30.0)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('mode', 'frontier')       # frontier | direct (old behaviour: send the goal at once)
        self.declare_parameter('max_time', 0.0)          # s (sim time) after which the test ends as 'timeout'; 0 = off
        self.declare_parameter('result_file', '')       # JSON summary written when the test ends
        self.declare_parameter('stall_timeout', 40.0)    # s (sim time) without moving; 0 disables
        self.declare_parameter('stall_distance', 0.15)   # m that counts as moving
        self.last_view = None
        self.t_start = None                              # sim time of the first goal sent
        self.path_length = 0.0
        self.last_sample = None
        self.n_frontier_goals = 0
        self.n_failed = 0
        self.create_timer(0.2, self.sample_path)
        self.last_move = None                            # (x, y, time) of last significant move
        self.goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)

        self.map = None
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        # Plan on the global costmap itself: it is exactly what the planner will see (inflation,
        # lidar marks, resize timing), so a frontier chosen here is plannable.
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.target = None            # (row, col) currently sent, or None
        self.target_is_goal = False
        self.seq = 0                  # id of the latest goal sent; older results are ignored
        self.blacklist = []           # (row, col, expiry)
        self.done = False
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'Goal {self.goal} (map frame); waiting for the global costmap and Nav2')

    def on_map(self, msg):
        self.map = msg

    # -- helpers -----------------------------------------------------------------------
    def to_cell(self, x, y):
        m = self.map
        return (int((y - m.info.origin.position.y) / m.info.resolution),
                int((x - m.info.origin.position.x) / m.info.resolution))

    def to_xy(self, cell):
        m = self.map
        return (m.info.origin.position.x + (cell[1] + 0.5) * m.info.resolution,
                m.info.origin.position.y + (cell[0] + 0.5) * m.info.resolution)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def send(self, cell, xy, robot, is_goal):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = xy
        yaw = math.atan2(xy[1] - robot[1], xy[0] - robot[0])
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.target, self.target_is_goal = cell, is_goal
        self.get_logger().info(
            f'{"GOAL" if is_goal else "frontier"} -> ({xy[0]:.2f}, {xy[1]:.2f})')
        self.seq += 1
        if self.t_start is None:
            self.t_start = self.now_s()
        if not is_goal:
            self.n_frontier_goals += 1
        seq = self.seq
        self.client.send_goal_async(goal_msg).add_done_callback(
            lambda f: self.on_accepted(f, seq))

    def on_accepted(self, future, seq):
        handle = future.result()
        if seq != self.seq:
            return                                      # superseded before it was accepted
        if not handle.accepted:
            self.fail_target('rejected')
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq != self.seq:
            return                                      # preempted by a newer goal: not a failure
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.target_is_goal:
                self.get_logger().info('GOAL REACHED')
                self.finish()
            self.target = None
        elif status != GoalStatus.STATUS_CANCELED:
            self.fail_target(f'status {status}')

    def fail_target(self, why):
        self.n_failed += 1
        if self.target is not None:
            self.get_logger().warn(f'target failed ({why}); blacklisting it')
            if not self.target_is_goal:
                expiry = time.time() + self.get_parameter('blacklist_seconds').value
                self.blacklist.append((*self.target, expiry))
        self.target = None

    def finish(self, outcome='reached'):
        self.write_result(outcome)
        self.done = True
        rclpy.try_shutdown()                            # the launch file ends the test with this node

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sample_path(self):
        if self.done or self.t_start is None:
            return
        pos = self.robot_xy()
        if pos is None:
            return
        if self.last_sample is not None:
            d = math.hypot(pos[0] - self.last_sample[0], pos[1] - self.last_sample[1])
            if d < 1.0:                                  # ignore a localisation jump
                self.path_length += d
        self.last_sample = pos

    def write_result(self, outcome):
        t = self.now_s() - self.t_start if self.t_start is not None else 0.0
        result = {'outcome': outcome, 'sim_time_s': round(t, 2),
                  'path_m': round(self.path_length, 2),
                  'avg_speed_mps': round(self.path_length / t, 3) if t > 0 else 0.0,
                  'frontier_goals': self.n_frontier_goals, 'failed_targets': self.n_failed,
                  'goal': list(self.goal)}
        self.get_logger().info(f'RESULT {json.dumps(result)}')
        path = self.get_parameter('result_file').value
        if path:
            with open(path, 'w') as f:
                json.dump(result, f)

    def check_stall(self, robot):
        """Stop the test when the robot has not moved for stall_timeout seconds."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_move is None or math.hypot(
                robot[0] - self.last_move[0],
                robot[1] - self.last_move[1]) > self.get_parameter('stall_distance').value:
            self.last_move = (robot[0], robot[1], now)
            return False
        timeout = self.get_parameter('stall_timeout').value
        if timeout > 0 and now - self.last_move[2] > timeout:
            self.get_logger().error(
                f'STALLED: robot at ({robot[0]:.2f}, {robot[1]:.2f}) has not moved '
                f'{timeout:.0f} s; stopping the test')
            self.dump_view(robot)
            self.write_result('stalled')
            self.done = True
            rclpy.try_shutdown()
            return True
        return False

    def dump_view(self, robot):
        """Save the costmap the explorer last used, for diagnosing the stall."""
        if self.last_view is None:
            return
        grid, res, ox, oy = self.last_view
        np.savez('/tmp/goal_explorer_stall.npz', grid=grid, res=res, origin=(ox, oy),
                 robot=robot, goal=self.goal,
                 blacklist=np.array([b[:2] for b in self.blacklist]).reshape(-1, 2))
        self.get_logger().info('saved /tmp/goal_explorer_stall.npz')

    # -- main loop ---------------------------------------------------------------------
    def tick(self):
        if self.done or self.map is None or not self.client.server_is_ready():
            return
        robot = self.robot_xy()
        if robot is None:
            return
        if self.check_stall(robot):
            return
        max_time = self.get_parameter('max_time').value
        if max_time > 0 and self.t_start is not None and self.now_s() - self.t_start > max_time:
            self.get_logger().error(f'TIMEOUT after {max_time:.0f} s; stopping the test')
            self.finish('timeout')
            return
        if math.hypot(robot[0] - self.goal[0], robot[1] - self.goal[1]) < \
                self.get_parameter('goal_tolerance').value:
            self.get_logger().info('GOAL REACHED')
            self.finish()
            return

        if self.get_parameter('mode').value == 'direct':
            # Old behaviour: hand the goal straight to Nav2, whatever the map looks like.
            if self.target is None and time.time() - getattr(self, 'last_direct', 0.0) > 5.0:
                self.last_direct = time.time()
                self.send((0, 0), self.goal, robot, True)
            return

        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        self.last_view = (grid, info.resolution, info.origin.position.x, info.origin.position.y)
        robot_cell, goal_cell = self.to_cell(*robot), self.to_cell(*self.goal)
        now = time.time()
        self.blacklist = [b for b in self.blacklist if b[2] > now]
        best, best_s, goal_ok, dist, safe = fl.best_frontier(
            grid, robot_cell, goal_cell, info.resolution,
            clearance_m=self.get_parameter('clearance').value, min_clearance_m=0.0,
            occ_thresh=99,
            min_size=self.get_parameter('min_frontier_cells').value,
            goal_weight=self.get_parameter('goal_weight').value,
            blacklist=[(b[0], b[1]) for b in self.blacklist])

        if goal_ok:
            if not self.target_is_goal or self.target is None:
                self.send(goal_cell, self.goal, robot, True)
            return
        if best is None:
            if self.target is None:
                self.get_logger().warn('no frontier left and goal unreachable', throttle_duration_sec=10.0)
            return

        # Keep the current frontier unless it is gone or clearly worse (prevents dithering).
        if self.target is not None and not self.target_is_goal:
            r, c = self.target
            still_valid = (0 <= r < dist.shape[0] and 0 <= c < dist.shape[1] and dist[r, c] >= 0
                           and fl.find_frontier_near(grid, safe, dist, self.target))
            if still_valid:
                cur = fl.score(dist, self.target, goal_cell, info.resolution,
                               self.get_parameter('goal_weight').value)
                if best_s > cur - self.get_parameter('switch_margin').value:
                    return
        self.send(best, self.to_xy(best), robot, False)


def main(args=None):
    rclpy.init(args=args)
    node = GoalExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
