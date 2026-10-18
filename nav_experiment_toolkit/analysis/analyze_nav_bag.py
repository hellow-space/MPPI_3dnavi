#!/usr/bin/env python3
"""
ROS2 Nav2 bag analyzer for the four-wheel-legged inspection robot.

Outputs:
- report.md
- metrics.csv
- trajectory.csv
- active_global_error.csv
- controller_reference_error.csv
- trajectory_map.png
- active_global_error.png
- controller_reference_error.png
- cmd_linear.png
- cmd_angular.png

Key definition:
1) active_global_error:
   actual pose p(t) vs latest /plan published at or before t.
   This handles global-plan refresh correctly.

2) initial_plan_error:
   actual pose p(t) vs the first /plan after the goal.
   This exposes whether frequent replanning is "following the robot".

3) controller_reference_error:
   actual pose p(t) vs latest /transformed_global_plan at or before t.
   This is the best metric for controller tracking behavior.

Actual pose:
- Primary: TF map -> base_link reconstructed from /tf + /tf_static.
- Fallback: /amcl_pose.
"""

import argparse
import bisect
import csv
import math
import os
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def qyaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """A->B composed with B->C gives A->C. tuples (x,y,yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        wrap(ath + bth),
    )


def inv(a):
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, wrap(-th))


def tf_tuple(tf):
    tr = tf.transform.translation
    return (float(tr.x), float(tr.y), qyaw(tf.transform.rotation))


def stamp_ns(stamp, fallback):
    ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return ns if ns > 0 else fallback


def stats(vals):
    a = np.asarray(vals, dtype=float)
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return {"mean": math.nan, "rmse": math.nan, "max": math.nan, "p95": math.nan}
    return {
        "mean": float(np.mean(a)),
        "rmse": float(np.sqrt(np.mean(a*a))),
        "max": float(np.max(a)),
        "p95": float(np.percentile(a, 95)),
    }


def nearest_distance(x, y, path_xy):
    if path_xy is None or len(path_xy) == 0:
        return math.nan
    d = path_xy - np.array([x, y], dtype=float)
    return float(np.sqrt(np.min(np.sum(d*d, axis=1))))


def path_length(xy):
    if len(xy) < 2:
        return 0.0
    d = np.diff(xy, axis=0)
    return float(np.sum(np.sqrt(np.sum(d*d, axis=1))))


class BagData:
    def __init__(self):
        self.tf_dyn = defaultdict(list)   # (parent, child) -> [(t, tf_tuple)]
        self.tf_static = {}
        self.amcl = []
        self.goals = []
        self.plans = []
        self.refs = []
        self.cmd_raw = []
        self.cmd_final = []
        self.map = None
        self.bag_start = None
        self.bag_end = None


def read_bag(bag_path):
    data = BagData()
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )

    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    wanted = {
        "/tf", "/tf_static", "/amcl_pose", "/goal_pose", "/plan",
        "/transformed_global_plan", "/command/cmd_twist", "/cmd_vel", "/map"
    }
    msg_types = {}
    for t in wanted:
        if t in topic_types:
            msg_types[t] = get_message(topic_types[t])

    while reader.has_next():
        topic, raw, t = reader.read_next()
        if data.bag_start is None:
            data.bag_start = t
        data.bag_end = t
        if topic not in msg_types:
            continue
        msg = deserialize_message(raw, msg_types[topic])

        if topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                parent = tr.header.frame_id.lstrip("/")
                child = tr.child_frame_id.lstrip("/")
                if not parent or not child:
                    continue
                ts = stamp_ns(tr.header.stamp, t)
                tup = tf_tuple(tr)
                if topic == "/tf_static":
                    data.tf_static[(parent, child)] = tup
                else:
                    data.tf_dyn[(parent, child)].append((ts, tup))

        elif topic == "/amcl_pose":
            p = msg.pose.pose
            ts = stamp_ns(msg.header.stamp, t)
            data.amcl.append((ts, float(p.position.x), float(p.position.y), qyaw(p.orientation)))

        elif topic == "/goal_pose":
            p = msg.pose
            ts = stamp_ns(msg.header.stamp, t)
            data.goals.append((ts, float(p.position.x), float(p.position.y), qyaw(p.orientation), msg.header.frame_id.lstrip("/")))

        elif topic in ("/plan", "/transformed_global_plan"):
            ts = stamp_ns(msg.header.stamp, t)
            frame = msg.header.frame_id.lstrip("/")
            pts = np.array([[float(p.pose.position.x), float(p.pose.position.y)] for p in msg.poses], dtype=float)
            item = (ts, frame, pts)
            if topic == "/plan":
                data.plans.append(item)
            else:
                data.refs.append(item)

        elif topic in ("/command/cmd_twist", "/cmd_vel"):
            item = (t, float(msg.linear.x), float(msg.angular.z))
            if topic == "/command/cmd_twist":
                data.cmd_raw.append(item)
            else:
                data.cmd_final.append(item)

        elif topic == "/map":
            if data.map is None:
                data.map = msg

    for k in data.tf_dyn:
        data.tf_dyn[k].sort(key=lambda z: z[0])
    data.amcl.sort()
    data.goals.sort()
    data.plans.sort(key=lambda z: z[0])
    data.refs.sort(key=lambda z: z[0])
    data.cmd_raw.sort()
    data.cmd_final.sort()
    return data


def latest_edge(series, t):
    times = [z[0] for z in series]
    i = bisect.bisect_right(times, t) - 1
    return None if i < 0 else series[i][1]


def tf_at(data, source, target, t):
    source = source.lstrip("/")
    target = target.lstrip("/")
    if source == target:
        return (0.0, 0.0, 0.0)

    graph = defaultdict(list)
    keys = set(data.tf_static.keys()) | set(data.tf_dyn.keys())
    for parent, child in keys:
        val = data.tf_static.get((parent, child))
        if (parent, child) in data.tf_dyn:
            dyn = latest_edge(data.tf_dyn[(parent, child)], t)
            if dyn is not None:
                val = dyn
        if val is None:
            continue
        graph[parent].append((child, val))
        graph[child].append((parent, inv(val)))

    q = deque([(source, (0.0, 0.0, 0.0))])
    seen = {source}
    while q:
        node, T = q.popleft()
        if node == target:
            return T
        for nxt, edge in graph[node]:
            if nxt in seen:
                continue
            seen.add(nxt)
            q.append((nxt, compose(T, edge)))
    return None


def transform_path_to_map(data, frame, pts, t, map_frame):
    frame = frame or map_frame
    if frame == map_frame:
        return pts
    T = tf_at(data, map_frame, frame, t)
    if T is None:
        return None
    x, y, th = T
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + np.array([x, y])


def actual_trajectory(data, map_frame, base_frame):
    # Use odometry cadence as sample times if available in TF itself is awkward;
    # sample at 10 Hz across bag. This provides a stable analysis grid.
    if data.bag_start is not None and data.bag_end is not None:
        ts = np.arange(data.bag_start, data.bag_end + 1, 100_000_000, dtype=np.int64)
        out = []
        for t in ts:
            T = tf_at(data, map_frame, base_frame, int(t))
            if T is not None:
                out.append((int(t), T[0], T[1], T[2], "tf"))
        if len(out) >= 5:
            return out

    # Fallback to AMCL.
    return [(t, x, y, yaw, "amcl") for t, x, y, yaw in data.amcl]


def select_window(traj, goals, xy_tol, yaw_tol):
    if not traj:
        return [], None, None, False
    if not goals:
        return traj, traj[0][0], traj[-1][0], False

    goal = goals[0]
    gt, gx, gy, gyaw, _ = goal
    traj2 = [p for p in traj if p[0] >= gt]
    if not traj2:
        return [], gt, None, False

    reached_t = None
    for t, x, y, yaw, src in traj2:
        if math.hypot(x-gx, y-gy) <= xy_tol and abs(wrap(yaw-gyaw)) <= yaw_tol:
            reached_t = t
            break

    end_t = reached_t if reached_t is not None else traj2[-1][0]
    used = [p for p in traj2 if p[0] <= end_t]
    return used, gt, end_t, reached_t is not None


def latest_plan(plan_times, plans, t):
    i = bisect.bisect_right(plan_times, t) - 1
    return None if i < 0 else plans[i]


def unique_plan_count(plans):
    seen = set()
    for _, _, pts in plans:
        if len(pts) == 0:
            continue
        # Geometry fingerprint robust to tiny floating differences.
        arr = np.round(pts[::max(1, len(pts)//50)], 2)
        seen.add(arr.tobytes())
    return len(seen)


def save_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="rosbag2 directory containing metadata.yaml")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--goal-tolerance", type=float, default=0.30)
    ap.add_argument("--yaw-tolerance", type=float, default=0.35)
    ap.add_argument("--max-global-plan-age", type=float, default=5.0,
                    help="seconds; ignore stale active global plan")
    ap.add_argument("--max-controller-ref-age", type=float, default=1.0,
                    help="seconds; ignore stale transformed_global_plan")
    ap.add_argument("-o", "--output", default=None)
    args = ap.parse_args()

    bag = Path(args.bag).resolve()
    if not (bag/"metadata.yaml").exists():
        raise SystemExit(f"Not a rosbag2 directory: {bag}")

    out = Path(args.output).resolve() if args.output else bag/"analysis"
    out.mkdir(parents=True, exist_ok=True)

    print("[1/6] Reading bag...")
    data = read_bag(bag)

    print("[2/6] Reconstructing map->base_link trajectory...")
    traj = actual_trajectory(data, args.map_frame, args.base_frame)
    traj, start_t, end_t, reached = select_window(
        traj, data.goals, args.goal_tolerance, args.yaw_tolerance
    )
    if len(traj) < 2:
        raise SystemExit("Could not reconstruct enough actual poses from TF or /amcl_pose.")

    t0 = traj[0][0]
    traj_xy = np.array([[p[1], p[2]] for p in traj], dtype=float)
    traj_rows = [((p[0]-t0)/1e9, p[1], p[2], p[3], p[4]) for p in traj]
    save_csv(out/"trajectory.csv", ["time_s","x","y","yaw","source"], traj_rows)

    print("[3/6] Preparing time-varying plans...")
    plans = []
    for t, frame, pts in data.plans:
        q = transform_path_to_map(data, frame, pts, t, args.map_frame)
        if q is not None and len(q):
            plans.append((t, q))
    refs = []
    for t, frame, pts in data.refs:
        q = transform_path_to_map(data, frame, pts, t, args.map_frame)
        if q is not None and len(q):
            refs.append((t, q))

    plan_times = [z[0] for z in plans]
    ref_times = [z[0] for z in refs]

    # First plan after goal/start = frozen initial plan.
    initial_plan = None
    for t, p in plans:
        if t >= t0:
            initial_plan = p
            break
    if initial_plan is None and plans:
        initial_plan = plans[0][1]

    active_errors = []
    init_errors = []
    ref_errors = []

    max_plan_age_ns = int(args.max_global_plan_age * 1e9)
    max_ref_age_ns = int(args.max_controller_ref_age * 1e9)

    for t, x, y, yaw, src in traj:
        lp = latest_plan(plan_times, plans, t)
        if lp is not None:
            pt, path = lp
            e = nearest_distance(x, y, path) if t-pt <= max_plan_age_ns else math.nan
        else:
            e = math.nan
        active_errors.append((t, e))

        ie = nearest_distance(x, y, initial_plan) if initial_plan is not None else math.nan
        init_errors.append((t, ie))

        lr = latest_plan(ref_times, refs, t)
        if lr is not None:
            rt, path = lr
            re = nearest_distance(x, y, path) if t-rt <= max_ref_age_ns else math.nan
        else:
            re = math.nan
        ref_errors.append((t, re))

    save_csv(
        out/"active_global_error.csv",
        ["time_s","error_m"],
        [((t-t0)/1e9, e) for t,e in active_errors]
    )
    save_csv(
        out/"controller_reference_error.csv",
        ["time_s","error_m"],
        [((t-t0)/1e9, e) for t,e in ref_errors]
    )

    print("[4/6] Computing metrics...")
    act = stats([e for _,e in active_errors])
    ini = stats([e for _,e in init_errors])
    ref = stats([e for _,e in ref_errors])

    nav_time = (traj[-1][0] - traj[0][0]) / 1e9
    plen = path_length(traj_xy)

    # Plan publish statistics inside navigation interval.
    nav_plans = [t for t,_ in plans if traj[0][0] <= t <= traj[-1][0]]
    if len(nav_plans) >= 2:
        plan_rate = (len(nav_plans)-1) / ((nav_plans[-1]-nav_plans[0])/1e9)
    else:
        plan_rate = 0.0

    # Commands in navigation window.
    def cmd_slice(seq):
        return [z for z in seq if traj[0][0] <= z[0] <= traj[-1][0]]
    raw = cmd_slice(data.cmd_raw)
    final = cmd_slice(data.cmd_final)

    def cmd_metrics(seq, prefix):
        if not seq:
            return {
                f"{prefix}_max_abs_v": math.nan,
                f"{prefix}_max_abs_w": math.nan,
                f"{prefix}_rms_w": math.nan,
                f"{prefix}_w_total_variation": math.nan,
            }
        v = np.array([z[1] for z in seq])
        w = np.array([z[2] for z in seq])
        return {
            f"{prefix}_max_abs_v": float(np.max(np.abs(v))),
            f"{prefix}_max_abs_w": float(np.max(np.abs(w))),
            f"{prefix}_rms_w": float(np.sqrt(np.mean(w*w))),
            f"{prefix}_w_total_variation": float(np.sum(np.abs(np.diff(w)))) if len(w)>1 else 0.0,
        }

    metrics = {
        "goal_reached_by_pose_tolerance": int(reached),
        "navigation_time_s": nav_time,
        "actual_path_length_m": plen,
        "plan_message_count_in_navigation": len(nav_plans),
        "plan_unique_geometry_count_total": unique_plan_count(data.plans),
        "plan_publish_rate_hz": plan_rate,

        "active_global_error_mean_m": act["mean"],
        "active_global_error_rmse_m": act["rmse"],
        "active_global_error_p95_m": act["p95"],
        "active_global_error_max_m": act["max"],

        "initial_plan_error_mean_m": ini["mean"],
        "initial_plan_error_rmse_m": ini["rmse"],
        "initial_plan_error_p95_m": ini["p95"],
        "initial_plan_error_max_m": ini["max"],

        "controller_reference_error_mean_m": ref["mean"],
        "controller_reference_error_rmse_m": ref["rmse"],
        "controller_reference_error_p95_m": ref["p95"],
        "controller_reference_error_max_m": ref["max"],
    }
    metrics.update(cmd_metrics(raw, "raw_cmd"))
    metrics.update(cmd_metrics(final, "final_cmd"))

    save_csv(out/"metrics.csv", ["metric","value"], list(metrics.items()))

    print("[5/6] Plotting...")
    # Trajectory + map + initial/final global plans
    fig, ax = plt.subplots(figsize=(10, 8))
    if data.map is not None:
        m = data.map
        arr = np.array(m.data, dtype=float).reshape(m.info.height, m.info.width)
        arr[arr < 0] = np.nan
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        extent = [ox, ox + m.info.width*m.info.resolution,
                  oy, oy + m.info.height*m.info.resolution]
        ax.imshow(arr, origin="lower", extent=extent, cmap="gray_r", alpha=0.55)

    ax.plot(traj_xy[:,0], traj_xy[:,1], label="Actual map→base_link")
    if initial_plan is not None:
        ax.plot(initial_plan[:,0], initial_plan[:,1], label="Initial global plan")
    if plans:
        ax.plot(plans[-1][1][:,0], plans[-1][1][:,1], label="Final global plan")
    if data.goals:
        gx, gy = data.goals[0][1], data.goals[0][2]
        ax.scatter([gx], [gy], label="Goal")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Navigation trajectory")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out/"trajectory_map.png", dpi=160)
    plt.close(fig)

    def plot_error(rows, title, filename):
        xs = [(t-t0)/1e9 for t,e in rows if math.isfinite(e)]
        ys = [e for t,e in rows if math.isfinite(e)]
        fig, ax = plt.subplots(figsize=(10,4))
        ax.plot(xs, ys)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("nearest-path error [m]")
        ax.set_title(title)
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(out/filename, dpi=160)
        plt.close(fig)

    plot_error(active_errors, "Error to active /plan (time-aligned)", "active_global_error.png")
    plot_error(ref_errors, "Error to active /transformed_global_plan (time-aligned)", "controller_reference_error.png")

    def plot_cmd(seq1, seq2, idx, ylabel, title, filename):
        fig, ax = plt.subplots(figsize=(10,4))
        if seq1:
            ax.plot([(t-t0)/1e9 for t,_,_ in seq1], [z[idx] for z in seq1], label="/command/cmd_twist")
        if seq2:
            ax.plot([(t-t0)/1e9 for t,_,_ in seq2], [z[idx] for z in seq2], label="/cmd_vel")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out/filename, dpi=160)
        plt.close(fig)

    # tuples are (t,v,w), index 1=v, index 2=w
    plot_cmd(raw, final, 1, "linear.x [m/s]", "Linear velocity command", "cmd_linear.png")
    plot_cmd(raw, final, 2, "angular.z [rad/s]", "Angular velocity command", "cmd_angular.png")

    print("[6/6] Writing report...")
    report = f"""# Navigation Bag Analysis

Bag: `{bag}`

## Main metrics

| Metric | Value |
|---|---:|
| Goal reached by pose tolerance | {reached} |
| Navigation time | {nav_time:.3f} s |
| Actual path length | {plen:.3f} m |
| /plan messages during navigation | {len(nav_plans)} |
| Estimated /plan publish rate | {plan_rate:.3f} Hz |
| Unique global-plan geometries | {unique_plan_count(data.plans)} |

## Tracking metrics

### 1. Controller tracking — recommended primary metric

Actual `map → base_link` position at time `t` is compared with the latest
`/transformed_global_plan` available at or before `t`.

| Metric | Value |
|---|---:|
| Mean | {ref['mean']:.4f} m |
| RMSE | {ref['rmse']:.4f} m |
| P95 | {ref['p95']:.4f} m |
| Max | {ref['max']:.4f} m |

### 2. Active global-plan error

Actual pose at time `t` is compared with the latest `/plan` available at or before `t`.
This explicitly handles global-plan refresh.

| Metric | Value |
|---|---:|
| Mean | {act['mean']:.4f} m |
| RMSE | {act['rmse']:.4f} m |
| P95 | {act['p95']:.4f} m |
| Max | {act['max']:.4f} m |

### 3. Initial-plan deviation

The first `/plan` after navigation start is frozen and used for the full run.
This metric is deliberately NOT updated when the planner replans.

| Metric | Value |
|---|---:|
| Mean | {ini['mean']:.4f} m |
| RMSE | {ini['rmse']:.4f} m |
| P95 | {ini['p95']:.4f} m |
| Max | {ini['max']:.4f} m |

## How to interpret the three errors

- Low controller-reference error = MPPI follows the reference segment it is currently given.
- Low active-global error = robot stays close to the currently active replanned global path.
- Initial-plan error much larger than active-global error = the global planner is changing the route around the robot; do not claim this as pure controller improvement.

## Command metrics note

`/command/cmd_twist` has multiple possible Nav2 publishers and `/cmd_vel` also has
`keep_alive_zero` plus `velocity_smoother`. rosbag2 stores the topic stream, not the publisher
identity. Therefore command plots are excellent for qualitative comparison, but exact
publisher-specific statistics require separating topics upstream.
"""
    (out/"report.md").write_text(report, encoding="utf-8")

    print()
    print("Done.")
    print(f"Results: {out}")
    print(f"Primary controller RMSE: {ref['rmse']:.4f} m")
    print(f"Active global-plan RMSE: {act['rmse']:.4f} m")
    print(f"Initial-plan RMSE: {ini['rmse']:.4f} m")


if __name__ == "__main__":
    main()
