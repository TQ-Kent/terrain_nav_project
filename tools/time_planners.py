"""Time ComputePathToPose per planner on the live global costmap (round trip, ms)."""
import json, math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

rclpy.init(); n = Node('time_planners', parameter_overrides=[])
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
buf = Buffer(); TransformListener(buf, n)
grid = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: grid.update(m=m), 1)
ac = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
ac.wait_for_server()
res = {p: [] for p in ('GridBased', 'Smac2D', 'ThetaStar', 'SmacHybrid')}
fails = {p: 0 for p in res}
t_end = time.time() + float(sys.argv[1]); 
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.2)
    if 'm' not in grid: continue
    try: t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
    except Exception: continue
    m = grid['m']; w, h, r = m.info.width, m.info.height, m.info.resolution
    a = np.array(m.data, dtype=np.int16).reshape(h, w)
    rx, ry = t.transform.translation.x, t.transform.translation.y
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    ys, xs = np.where((a >= 0) & (a < 30))
    d = np.hypot(ox + xs * r - rx, oy + ys * r - ry)
    ok = np.where((d > 3.0) & (d < 8.0))[0]
    if len(ok) == 0: continue
    i = ok[len(ok) // 2]
    for p in res:
        g = ComputePathToPose.Goal(); g.planner_id = p; g.use_start = False
        g.goal = PoseStamped(); g.goal.header.frame_id = 'map'
        g.goal.pose.position.x = float(ox + xs[i] * r); g.goal.pose.position.y = float(oy + ys[i] * r)
        g.goal.pose.orientation.w = 1.0
        t0 = time.time()
        fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        gh = fut.result()
        if gh is None or not gh.accepted: fails[p] += 1; continue
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=10)
        if rf.result() is None or rf.result().status != 4 or len(rf.result().result.path.poses) == 0:
            fails[p] += 1; continue
        res[p].append(((time.time() - t0) * 1000, len(rf.result().result.path.poses) * r, int(d[ok].max() > 0)))
out = {p: {'n': len(v), 'fail': fails[p],
           'ms_median': float(np.median([x[0] for x in v])) if v else None,
           'ms_p95': float(np.percentile([x[0] for x in v], 95)) if v else None,
           'ms_max': float(max(x[0] for x in v)) if v else None} for p, v in res.items()}
print(json.dumps(out, indent=1))
