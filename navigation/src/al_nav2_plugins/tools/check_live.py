#!/usr/bin/env python3
# Copyright 2026 Matvey Razgulov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Live checks for the al_nav2_plugins (T-009e). Run through check_live.sh.

Needs a running Nav2 (planner_server, global_costmap, TF map->base_link).
Prints one line per check: PASS / FAIL / WARN / INFO / SKIP. Exit code 1 if any FAIL.
Thresholds marked [heuristic] are engineering judgement, not spec values.
"""

import argparse
import math
import sys
import time

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def heading_deviation(poses):
    """Return [(remaining_m, |yaw - travel direction|)] for poses 0..n-2 (xy, yaw tuples)."""
    n = len(poses)
    rem = [0.0] * n
    for i in range(n - 2, -1, -1):
        rem[i] = rem[i + 1] + math.hypot(poses[i + 1][0] - poses[i][0],
                                         poses[i + 1][1] - poses[i][1])
    out = []
    for i in range(n - 1):
        dx = poses[i + 1][0] - poses[i][0]
        dy = poses[i + 1][1] - poses[i][1]
        if math.hypot(dx, dy) < 1e-6:
            continue
        out.append((rem[i], abs(wrap_angle(poses[i][2] - math.atan2(dy, dx)))))
    return out


def median(values):
    s = sorted(values)
    if not s:
        return float('nan')
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def cell_cost(grid, x, y):
    """grid: dict(res, ox, oy, nx, ny, data). Returns the cell cost or None outside."""
    mx = int(math.floor((x - grid['ox']) / grid['res'] + 1e-9))
    my = int(math.floor((y - grid['oy']) / grid['res'] + 1e-9))
    if mx < 0 or my < 0 or mx >= grid['nx'] or my >= grid['ny']:
        return None
    return int(grid['data'][my * grid['nx'] + mx])


def parse_floats(text, n_min):
    v = [float(t) for t in text.replace(';', ',').split(',') if t.strip()]
    if len(v) < n_min:
        raise argparse.ArgumentTypeError('expected at least %d numbers: %r' % (n_min, text))
    return v


class Report:
    def __init__(self):
        self.fails = 0
        self.warns = 0

    def line(self, kind, text):
        if kind == 'FAIL':
            self.fails += 1
        elif kind == 'WARN':
            self.warns += 1
        print('%-4s %s' % (kind, text), flush=True)

    def check(self, ok, text, bad_kind='FAIL'):
        self.line('PASS' if ok else bad_kind, text)
        return ok


def load_marker_xy(path, ids):
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    out = {}
    for m in doc.get('big_markers', []):
        if not ids or int(m['id']) in ids:
            out[int(m['id'])] = (float(m['position']['x']), float(m['position']['y']))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expect', choices=['active', 'control', 'baseline'], default='active')
    p.add_argument('--goal', type=lambda s: parse_floats(s, 2), help='x,y[,yaw] in map')
    p.add_argument('--scenario', help='scenario YAML; the goal farthest from the robot is used')
    p.add_argument('--cell', action='append', default=[],
                   type=lambda s: parse_floats(s, 4),
                   help='x,y,min,max: layer-included global costmap cost must be in [min,max]')
    p.add_argument('--c-max', type=int, default=0, help='loc_layer.c_max (from param get)')
    p.add_argument('--landmarks-file', default='')
    p.add_argument('--marker-ids', default='')
    p.add_argument('--marker-normal', default='0,-1')
    p.add_argument('--view-dist', type=float, default=1.88,
                   help='distance in front of a marker for the default view cell [m]')
    p.add_argument('--step', type=float, default=0.20, help='GridBased.step (path spacing)')
    p.add_argument('--min-dev', type=float, default=0.30,
                   help='[heuristic] active: required max |yaw - tangent| [rad]')
    p.add_argument('--ctl-dev', type=float, default=0.20,
                   help='[heuristic] control: allowed median |yaw - tangent| [rad]')
    p.add_argument('--tail', type=float, default=1.0,
                   help='ignore poses closer than this to the goal (goal yaw is pinned) [m]')
    p.add_argument('--planner', default='GridBased')
    p.add_argument('--action', default='/compute_path_to_pose')
    p.add_argument('--costmap-topic', default='/global_costmap/costmap_raw')
    p.add_argument('--timeout', type=float, default=20.0)
    p.add_argument('--no-sim-time', action='store_true')
    args = p.parse_args()

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.msg import Costmap
    from tf2_ros import Buffer, TransformListener

    rep = Report()
    rclpy.init(args=None if args.no_sim_time else ['--ros-args', '-p', 'use_sim_time:=true'])
    node = rclpy.create_node('al_check_live')

    def spin_until(pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            rclpy.spin_once(node, timeout_sec=0.1)
        return pred()

    # ---- robot pose
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    robot = {}

    def have_tf():
        try:
            t = tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001 (tf2 raises several types)
            return False
        robot['x'] = t.transform.translation.x
        robot['y'] = t.transform.translation.y
        robot['yaw'] = quat_to_yaw(t.transform.rotation)
        return True

    if not rep.check(spin_until(have_tf, args.timeout), 'TF map->base_link available'):
        print('FAIL cannot continue without the robot pose')
        return 1
    rep.line('INFO', 'robot at (%.2f, %.2f), yaw %.2f rad' % (robot['x'], robot['y'], robot['yaw']))

    # ---- costmap snapshot
    grid = {}

    def on_costmap(msg):
        md = msg.metadata
        grid.update(res=md.resolution, ox=md.origin.position.x, oy=md.origin.position.y,
                    nx=md.size_x, ny=md.size_y, data=bytes(msg.data))

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Costmap, args.costmap_topic, on_costmap, qos)
    got_map = spin_until(lambda: bool(grid), args.timeout)
    rep.check(got_map, 'global costmap received on %s' % args.costmap_topic)

    if got_map and args.expect != 'baseline' and args.c_max > 0:
        data = grid['data']
        n_cmax = sum(1 for v in data if v == args.c_max)
        distinct = len({v for v in data if 0 < v <= args.c_max})
        rep.line('INFO', 'cells == c_max (%d): %d; distinct costs in 1..c_max: %d'
                 % (args.c_max, n_cmax, distinct))
        if args.expect == 'active':
            rep.check(n_cmax >= 500, '[heuristic] >= 500 free cells at the layer maximum '
                      '(unobserved area costs c_max)', 'WARN')
            rep.check(distinct >= 5, '[heuristic] layer produces a graded cost (>= 5 distinct '
                      'values in 1..c_max)', 'WARN')

    # ---- explicit and default cells
    cells = list(args.cell)
    if got_map and args.expect == 'active' and args.landmarks_file and args.c_max > 0:
        try:
            ids = {int(t) for t in args.marker_ids.replace(';', ',').split(',') if t.strip()}
            markers = load_marker_xy(args.landmarks_file, ids)
            nrm = parse_floats(args.marker_normal, 2)
            norm = math.hypot(nrm[0], nrm[1]) or 1.0
            for mid in sorted(markers)[:2]:
                mx, my = markers[mid]
                x = mx + args.view_dist * nrm[0] / norm
                y = my + args.view_dist * nrm[1] / norm
                c = cell_cost(grid, x, y)
                if c is None or c >= 253:
                    rep.line('SKIP', 'view cell of marker %d at (%.2f, %.2f): cost %s (not free)'
                             % (mid, x, y, c))
                else:
                    rep.check(c <= 0.9 * args.c_max,
                              '[heuristic] marker %d seen from %.2f m: cost %d <= 0.9*c_max'
                              % (mid, args.view_dist, c))
        except Exception as exc:  # noqa: BLE001
            rep.line('WARN', 'default view cells skipped: %s' % exc)
    for x, y, lo, hi in cells:
        c = cell_cost(grid, x, y) if got_map else None
        rep.check(c is not None and lo <= c <= hi,
                  'cell (%.2f, %.2f) cost %s in [%g, %g]' % (x, y, c, lo, hi))

    # ---- plan sample
    goal = None
    if args.goal:
        goal = (args.goal[0], args.goal[1], args.goal[2] if len(args.goal) > 2 else 0.0)
    elif args.scenario:
        import yaml
        with open(args.scenario) as f:
            gl = yaml.safe_load(f)['goals']
        g = max(gl, key=lambda q: math.hypot(q['x'] - robot['x'], q['y'] - robot['y']))
        goal = (float(g['x']), float(g['y']), float(g.get('yaw', 0.0)))
    if goal is None:
        rep.line('SKIP', 'plan sample: give --goal x,y[,yaw] or --scenario FILE')
    else:
        client = ActionClient(node, ComputePathToPose, args.action)
        if not rep.check(client.wait_for_server(timeout_sec=args.timeout),
                         'action server %s up' % args.action):
            return 1
        req = ComputePathToPose.Goal()
        req.goal = PoseStamped()
        req.goal.header.frame_id = 'map'
        qx, qy, qz, qw = yaw_to_quat(goal[2])
        req.goal.pose.position.x, req.goal.pose.position.y = goal[0], goal[1]
        req.goal.pose.orientation.x, req.goal.pose.orientation.y = qx, qy
        req.goal.pose.orientation.z, req.goal.pose.orientation.w = qz, qw
        req.use_start = False
        req.planner_id = args.planner
        fut = client.send_goal_async(req)
        spin_until(fut.done, args.timeout)
        handle = fut.result() if fut.done() else None
        if not rep.check(handle is not None and handle.accepted, 'ComputePathToPose goal accepted'):
            return 1
        res_fut = handle.get_result_async()
        spin_until(res_fut.done, 3 * args.timeout)
        rep.check(res_fut.done(), 'plan returned')
        if res_fut.done():
            path = res_fut.result().result.path
            poses = [(ps.pose.position.x, ps.pose.position.y, quat_to_yaw(ps.pose.orientation))
                     for ps in path.poses]
            rep.line('INFO', 'goal (%.2f, %.2f, %.2f) -> %d poses' % (goal + (len(poses),)))
            if rep.check(len(poses) >= 2, 'path has >= 2 poses'):
                gx, gy, gyaw = poses[-1]
                rep.check(math.hypot(gx - goal[0], gy - goal[1]) < 1e-3 and
                          abs(wrap_angle(gyaw - goal[2])) < 0.02,
                          'last pose equals the requested goal (xy 1 mm, yaw 0.02 rad)')
                if args.expect != 'baseline':
                    seg = [math.hypot(poses[i + 1][0] - poses[i][0], poses[i + 1][1] - poses[i][1])
                           for i in range(len(poses) - 1)]
                    rep.check(abs(median(seg) - args.step) < 0.05,
                              'median pose spacing %.3f m ~ step %.2f m' % (median(seg), args.step))
                    rep.check(abs(wrap_angle(poses[0][2] - robot['yaw'])) < 0.2,
                              'first pose yaw %.2f ~ robot yaw %.2f (start pinned)'
                              % (poses[0][2], robot['yaw']), 'WARN')
                dev = [d for rem, d in heading_deviation(poses) if rem > args.tail]
                if dev:
                    mx_d, md_d = max(dev), median(dev)
                    frac = sum(1 for d in dev if d > 0.1) / float(len(dev))
                    rep.line('INFO', '|yaw - tangent| (poses > %.1f m from goal, n=%d): '
                             'max %.2f, median %.2f rad, %.0f%% > 0.1 rad'
                             % (args.tail, len(dev), mx_d, md_d, 100 * frac))
                    if args.expect == 'active':
                        rep.check(mx_d >= args.min_dev and frac >= 0.1,
                                  '[heuristic] yaw differs from the path tangent (max >= %.2f '
                                  'rad, >= 10%% of poses) - choose a goal whose route passes '
                                  'markers' % args.min_dev)
                    elif args.expect == 'control':
                        rep.check(md_d <= args.ctl_dev,
                                  '[heuristic] control: headings follow the tangent '
                                  '(median <= %.2f rad)' % args.ctl_dev)
                else:
                    rep.line('SKIP', 'heading statistics: path shorter than --tail')

    rep.line('INFO', 'fails=%d warns=%d' % (rep.fails, rep.warns))
    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fails else 0


if __name__ == '__main__':
    sys.exit(main())
