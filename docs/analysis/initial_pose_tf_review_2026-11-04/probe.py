#!/usr/bin/env python3
"""Exercise existing initial_pose_tf callbacks without ROS init/nodes/publishers."""
import importlib.util
import json
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
spec = importlib.util.spec_from_file_location('reviewed_initial_pose_tf', ROOT/'scripts/initial_pose_tf.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)  # main guard prevents ROS initialization.


class Logger:
    def __init__(self): self.messages = []
    def info(self, text): self.messages.append(('info', text))
    def error(self, text): self.messages.append(('error', text))


def transform(parent, child, seconds, matrix):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec = seconds
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, matrix[:3,3])
    q = Rotation.from_matrix(matrix[:3,:3]).as_quat()
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
    return t


def invoke(camera_base, frame='map', stamp=100, tf_stamp=1, x=3., y=4., quat=None):
    buffer = Buffer()  # In-memory TF only; no listener, node, service or transport.
    buffer.set_transform(transform('camera_init','base_link',tf_stamp,camera_base), 'offline_probe')
    logger = Logger()
    fake = SimpleNamespace(tf_buffer=buffer, map_to_camera=None, get_logger=lambda:logger)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id, pose.header.stamp.sec = frame, stamp
    pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
    q = [0.,0.,0.,1.] if quat is None else quat
    pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = q
    module.InitialPoseTF.initial_pose_callback(fake, pose)
    return fake.map_to_camera, logger.messages


rng = np.random.default_rng(20261104)
roundtrip_errors, quat_errors = [], []
for _ in range(2000):
    camera_base = np.eye(4)
    camera_base[:3,:3] = Rotation.random(random_state=rng).as_matrix()
    camera_base[:3,3] = rng.normal(size=3)
    target = np.eye(4)
    target[:3,:3] = Rotation.from_euler('z',rng.uniform(-np.pi,np.pi)).as_matrix()
    target[:2,3] = rng.normal(size=2)
    alignment = target @ np.linalg.inv(camera_base)
    roundtrip_errors.append(np.max(np.abs(alignment @ camera_base-target)))
    q = module.rot_to_quat(camera_base[:3,:3])
    quat_errors.append(np.max(np.abs(module.quat_to_rot(*q)-camera_base[:3,:3])))
assert max(roundtrip_errors)<1e-12 and max(quat_errors)<1e-12

identity = np.eye(4)
normal, _ = invoke(identity)
wrong_frame, frame_logs = invoke(identity, frame='odom')
stale, stale_logs = invoke(identity, stamp=100, tf_stamp=1)
assert np.array_equal(normal, wrong_frame) and np.array_equal(normal, stale)
assert all(level!='error' for level,_ in frame_logs+stale_logs)

# Clicking on a pitched body defines a tilted map even if the physical floor is level.
pitched_base = np.eye(4)
pitched_base[:3,:3] = Rotation.from_euler('y',10,degrees=True).as_matrix()
pitched_base[2,3] = .5
alignment, _ = invoke(pitched_base,x=0.,y=0.)
floor_points = np.array([[0.,0.,0.,1.],[2.,0.,0.,1.]])
mapped_floor = (alignment @ floor_points.T).T
level_base = np.eye(4);level_base[2,3] = .5
after_level = alignment @ level_base
tilt_example = dict(body_pitch_at_click_deg=10.,
    floor_height_change_over_2m=float(mapped_floor[1,2]-mapped_floor[0,2]),
    base_rpy_after_level_deg=Rotation.from_matrix(after_level[:3,:3]).as_euler('xyz',degrees=True).tolist())

# Latest-common-time demonstration: a constant transform published dynamically
# can force a lookup to an older timestamp than the latest moving transform.
buffer = Buffer()
for seconds in [8,10]:
    moving = np.eye(4);moving[0,3] = seconds
    buffer.set_transform(transform('camera_init','base_link',seconds,moving),'offline_probe')
buffer.set_transform(transform('map','camera_init',9,identity),'offline_probe')
common = buffer.lookup_transform('map','base_link',Time())
assert common.header.stamp.sec==9 and common.transform.translation.x==9.

recording = None
cache = Path('/tmp/mppi_indoor_spin_201946_data.pkl')
if cache.exists():
    d = pickle.load(cache.open('rb'))
    a,b = d['tf'][('camera_init','aft_mapped')],d['tf'][('map','camera_init')]
    times = np.asarray(d['times']['/trajectories'])
    ia,ib = np.searchsorted(a[:,0],times,side='right')-1,np.searchsorted(b[:,0],times,side='right')-1
    valid = (ia>=0)&(ib>=0)
    extra = np.maximum(0,a[ia[valid],1]-b[ib[valid],1])
    recording = dict(map_camera_translation_span=np.ptp(b[:,2:5],axis=0).tolist(),
        map_camera_quaternion_span=np.ptp(b[:,6:10],axis=0).tolist(),
        recorder_estimated_extra_common_time_age_s=dict(zip(['p50','p95','max'],map(float,np.percentile(extra,[50,95,100])))),
        caveat='Recorder reception approximation, not exact controller buffer contents; excludes internal sensor processing age.')

result = dict(random_rigid_transform_cases=2000,max_composition_error=float(max(roundtrip_errors)),
    max_quaternion_roundtrip_error=float(max(quat_errors)),
    non_map_initialpose_accepted_unchanged=True,
    pose_time_100s_and_tf_time_1s_accepted=True,
    tilt_example=tilt_example,
    latest_common_time_demo=dict(latest_moving_stamp=10,constant_edge_stamp=9,result_stamp=9),
    old_bag=recording,
    no_ros_nodes_or_publishers_created=True)
(OUT/'metrics.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
