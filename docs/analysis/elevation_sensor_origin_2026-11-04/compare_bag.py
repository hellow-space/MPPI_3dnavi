#!/usr/bin/env python3
"""Read-only old-bag geometry/gate comparison; no ROS init or publishers.

All recorded TF is available offline, unlike a live callback. This measures the
origin correction using recorded localization, not latency or terrain accuracy.
"""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from scipy.spatial.transform import Rotation
import yaml


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
ELEVATION = Path('/home/nvidia/elevation_map/src/elevation_mapping_cupy/elevation_mapping_cupy')


def rt(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return (Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix().astype(np.float32),
            np.array([t.x, t.y, t.z], dtype=np.float32))


def stats(values):
    return dict(zip(['min', 'p50', 'p95', 'max'],
                    map(float, np.percentile(values, [0, 50, 95, 100]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=ROOT / 'nav_experiment_toolkit/bags/nav_elevation/nav_elevation_2026-11-02_20-19-46')
    args = parser.parse_args()
    path = ELEVATION / 'elevation_mapping_cupy/pointcloud_geometry.py'
    spec = importlib.util.spec_from_file_location('pointcloud_geometry', path)
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    params = yaml.safe_load((ELEVATION / 'config/core/core_param.yaml').read_text())['/elevation_mapping_node']['ros__parameters']
    db = next(args.bag.glob('*.db3'))
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {name: ident for ident, name in con.execute('SELECT id,name FROM topics')}
    buffer = Buffer(cache_time=Duration(seconds=180.))
    ignored_empty_frame_transforms = 0
    for name in ['/tf_static', '/tf']:
        for (raw,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topics[name],)):
            for transform in deserialize_message(bytes(raw), TFMessage).transforms:
                if not transform.header.frame_id or not transform.child_frame_id:
                    ignored_empty_frame_transforms += 1
                    continue
                if name == '/tf_static':
                    buffer.set_transform_static(transform, 'offline_recording')
                else:
                    buffer.set_transform(transform, 'offline_recording')
    rows = con.execute('SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (topics['/cloud_registered'],)).fetchall()
    selected = np.unique(np.linspace(0, len(rows)-1, 48, dtype=int))
    results, rejected = [], []
    first = rows[0][1]
    for index in selected:
        ident, receipt = rows[index]
        raw = con.execute('SELECT data FROM messages WHERE id=?', (ident,)).fetchone()[0]
        msg = deserialize_message(bytes(raw), PointCloud2)
        stamp = Time.from_msg(msg.header.stamp)
        try:
            cloud_r, cloud_t = rt(buffer.lookup_transform('map', msg.header.frame_id, stamp))
            sensor_r, origin = rt(buffer.lookup_transform('map', 'livox_frame', stamp))
            _, base = rt(buffer.lookup_transform('map', 'base_link', stamp))
        except Exception as error:
            rejected.append({'frame_index': int(index), 'reason': str(error)})
            continue
        fields = {f.name: f for f in msg.fields}
        assert not msg.is_bigendian
        assert all(fields[n].datatype == 7 and fields[n].count == 1 for n in ['x', 'y', 'z'])
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                          'offsets': [fields[n].offset for n in ['x', 'y', 'z']], 'itemsize': msg.point_step})
        structured = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                strides=(msg.row_step, msg.point_step)).reshape(-1)
        cloud = np.column_stack([structured[n] for n in ['x', 'y', 'z']])
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        fixed, fixed_r, fixed_t = geometry.sensor_frame_points(cloud, cloud_r, cloud_t, sensor_r, origin)
        world = cloud @ cloud_r.T + cloud_t
        corrected_world = fixed @ fixed_r.T + fixed_t
        roi = (np.abs(world[:, :2] - base[:2]) < 3.5).all(axis=1)

        def gate(sensor):
            relative = world - sensor
            distance_sq = np.sum(relative**2, axis=1)
            horizontal = np.linalg.norm(relative[:, :2], axis=1)
            ramp = np.maximum(horizontal-params['ramped_height_range_b'], 0.)*params['ramped_height_range_a']+params['ramped_height_range_c']
            return ((distance_sq >= params['min_valid_distance']**2) &
                    (relative[:, 2] <= ramp) & (relative[:, 2] <= params['max_height_range']))

        old_gate, new_gate = gate(cloud_t), gate(origin)
        common = roi & old_gate & new_gate
        old_noise = params['sensor_noise_factor'] * np.sum(cloud**2, axis=1)
        new_noise = params['sensor_noise_factor'] * np.sum(fixed**2, axis=1)
        result = dict(frame_index=int(index), receipt_time_s=(receipt-first)*1e-9,
                      input_points=len(cloud), roi_points=int(roi.sum()),
                      sensor_origin_error_before_m=float(np.linalg.norm(cloud_t-origin)),
                      sensor_origin_error_after_m=float(np.linalg.norm(fixed_t-origin)),
                      world_geometry_max_change_m=float(np.max(np.linalg.norm(corrected_world-world, axis=1))),
                      gate_passed_before_in_roi=int(np.sum(roi & old_gate)),
                      gate_passed_after_in_roi=int(np.sum(roi & new_gate)),
                      gate_decisions_changed_in_roi=int(np.sum(roi & (old_gate != new_gate))),
                      both_gate_passed_in_roi=int(common.sum()))
        if common.any():
            result.update(mean_measurement_variance_before_m2=float(old_noise[common].mean()),
                          mean_measurement_variance_after_m2=float(new_noise[common].mean()),
                          mean_variance_ratio_before_to_after=float(old_noise[common].mean()/new_noise[common].mean()))
        results.append(result)
    assert results, rejected
    def aggregate(group):
        return {key: stats([r[key] for r in group if key in r]) for key in results[0] if key not in ['frame_index', 'receipt_time_s']}

    moved = [r for r in results if r['sensor_origin_error_before_m'] > 1.]
    metrics = dict(bag=str(args.bag), total_frames=len(rows), sampled_frames=len(selected),
                   compared_frames=len(results), frames_without_exact_tf=rejected,
                   ignored_empty_frame_transforms=ignored_empty_frame_transforms,
                   map_frame='map', sensor_frame='livox_frame', roi_size_m=7.,
                   parameters={name: params[name] for name in ['sensor_noise_factor', 'min_valid_distance', 'max_height_range', 'ramped_height_range_a', 'ramped_height_range_b', 'ramped_height_range_c']},
                   stats=aggregate(results),
                   frames_with_origin_displacement_over_1m=len(moved),
                   displaced_frame_stats=aggregate(moved) if moved else {},
                   scope='Offline CPU reproduction of existing distance/height gates and noise formula with recorded TF. Full-bag TF available; no live timestamp rejection, GPU map replay, terrain ground truth, or navigation validation. Variance means model measurement uncertainty, not measured height error.')
    (OUT / 'bag_comparison.json').write_text(json.dumps(metrics, indent=2)+'\n')
    with (OUT / 'bag_comparison_frames.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
