"""Run real initialization callbacks with an in-memory TF buffer, no ROS nodes."""
from collections import deque
import importlib.util
import math
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import Buffer

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
spec=importlib.util.spec_from_file_location('initial_pose_tf_under_test',ROOT/'scripts/initial_pose_tf.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def transform(seconds, degrees=10., x=0.):
    t=TransformStamped()
    t.header.frame_id='camera_init';t.child_frame_id='base_link'
    t.header.stamp.sec=seconds
    t.transform.translation.x=x;t.transform.translation.z=.5
    t.transform.rotation.y=math.sin(math.radians(degrees)/2)
    t.transform.rotation.w=math.cos(math.radians(degrees)/2)
    return t


def harness(degrees=10.,tf_stamp=10):
    f=SimpleNamespace(map_to_camera=None,tf_buffer=Buffer(),imu_samples=deque(maxlen=500),
        map_frame='map',odom_frame='camera_init',base_frame='base_link',
        max_data_age=.5,initial_pose_max_age=1.,window_seconds=1.,min_samples=20,
        pair_tolerance=.05,max_gyro=.08,max_acc_cv=.03,max_direction_spread_deg=2.,
        max_translation=.02,max_rotation_deg=2.,initial_base_height=0.,
        last_clock_ns=0,last_imu_stamp_ns=0,logs=[])
    f.get_logger=lambda:SimpleNamespace(info=lambda x:f.logs.append(x),
                                       warning=lambda x:f.logs.append(x),
                                       error=lambda x:f.logs.append(x))
    f.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=10_100_000_000,clock_type=ClockType.ROS_TIME),
                                      clock_type=ClockType.ROS_TIME)
    for name,method in vars(module.InitialPoseTF).items():
        if isinstance(method,staticmethod):
            setattr(f,name,method.__func__)
        elif callable(method) and name!='__init__':
            setattr(f,name,MethodType(method,f))
    for stamp in sorted(set([max(1,tf_stamp-1),tf_stamp])):
        f.tf_buffer.set_transform(transform(stamp,degrees),'test')
    static=TransformStamped();static.header.frame_id='base_link';static.child_frame_id='imu'
    static.transform.rotation.w=1.
    f.tf_buffer.set_transform_static(static,'test')
    a=math.radians(degrees)
    for i in range(900,1001):
        f.imu_samples.append(SimpleNamespace(stamp_ns=i*10_000_000,frame='imu',
            acceleration=np.array([-math.sin(a),0.,math.cos(a)])*9.81,
            angular_velocity=np.zeros(3)))
    return f


def initial_pose(frame='map'):
    m=PoseWithCovarianceStamped();m.header.frame_id=frame;m.header.stamp.sec=10
    m.pose.pose.position.x=3.;m.pose.pose.position.y=4.;m.pose.pose.orientation.w=1.
    return m


class InitialPoseCallbackTest(unittest.TestCase):
    def test_pitched_body_preserves_horizontal_map(self):
        f=harness();f.initial_pose_callback(initial_pose())
        self.assertIsNotNone(f.map_to_camera,f.logs)
        np.testing.assert_allclose(f.map_to_camera[:3,:3],np.eye(3),atol=1e-10)

    def test_non_map_pose_is_rejected(self):
        f=harness();f.initial_pose_callback(initial_pose('odom'))
        self.assertIsNone(f.map_to_camera)

    def test_stale_tf_is_rejected(self):
        f=harness(tf_stamp=1);f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_missing_imu_is_rejected(self):
        f=harness();f.imu_samples.clear();f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_rotating_robot_is_rejected(self):
        f=harness()
        for s in f.imu_samples:s.angular_velocity[:]=[0.,0.,.3]
        f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_tf_translation_motion_is_rejected(self):
        f=harness();f.tf_buffer.set_transform(transform(10,x=.2),'test')
        f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_missing_tf_does_not_crash(self):
        f=harness();f.tf_buffer=Buffer();f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_invalid_pose_preserves_previous_good_alignment(self):
        f=harness();f.initial_pose_callback(initial_pose());previous=f.map_to_camera.copy()
        m=initial_pose();m.pose.pose.position.x=float('nan')
        f.initial_pose_callback(m)
        np.testing.assert_array_equal(f.map_to_camera,previous)

    def test_clock_backwards_invalidates_alignment(self):
        f=harness();f.initial_pose_callback(initial_pose());self.assertIsNotNone(f.map_to_camera)
        f.last_clock_ns=12_000_000_000
        # Publish callback must invalidate before it ever tries to send a transform.
        f.publish_tf()
        self.assertIsNone(f.map_to_camera)

    def test_stale_raw_imu_reports_age_instead_of_silently_disappearing(self):
        f=harness();f.imu_samples.clear()
        msg=Imu();msg.header.frame_id='imu';msg.header.stamp.sec=1
        msg.linear_acceleration.z=9.81
        f.imu_callback(msg);f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)
        self.assertTrue(any('raw IMU' in line and '9.10' in line for line in f.logs),f.logs)

    def test_tilted_camera_is_compensated_without_removing_body_pitch(self):
        f=harness(degrees=-24.)
        a=math.radians(10.)
        for s in f.imu_samples:s.acceleration[:]=[-math.sin(a)*9.81,0.,math.cos(a)*9.81]
        f.initial_pose_callback(initial_pose());self.assertIsNotNone(f.map_to_camera,f.logs)
        self.assertAlmostEqual(f.map_to_camera[0,2],math.sin(math.radians(34.)))
        mapped=f.map_to_camera @ module.tf_to_matrix(transform(10,degrees=-24.))
        self.assertAlmostEqual(mapped[2,0],-math.sin(math.radians(10.)))

    def test_reinitialization_retains_established_vertical_reference(self):
        f=harness();f.initial_pose_callback(initial_pose());first=f.map_to_camera.copy()
        f.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=12_100_000_000,clock_type=ClockType.ROS_TIME),
                                          clock_type=ClockType.ROS_TIME)
        for stamp in [11,12]:
            t=transform(stamp,degrees=-8.,x=2.);t.transform.translation.z=.8
            f.tf_buffer.set_transform(t,'test')
        f.imu_samples.clear()
        a=math.radians(-8.)
        for i in range(1100,1201):
            f.imu_samples.append(SimpleNamespace(stamp_ns=i*10_000_000,frame='imu',
                acceleration=np.array([-math.sin(a),0.,math.cos(a)])*9.81,
                angular_velocity=np.zeros(3)))
        m=initial_pose();m.header.stamp.sec=12;m.pose.pose.orientation.z=math.sqrt(.5)
        m.pose.pose.orientation.w=math.sqrt(.5)
        f.initial_pose_callback(m)
        np.testing.assert_allclose(f.map_to_camera[2],first[2],atol=1e-10)
        mapped=f.map_to_camera @ module.tf_to_matrix(t)
        np.testing.assert_allclose(mapped[:3,3],[3.,4.,.3],atol=1e-10)
        self.assertAlmostEqual(math.atan2(mapped[1,0],mapped[0,0]),math.pi/2)

    def test_explicit_reset_drops_alignment_and_old_samples(self):
        f=harness();f.initial_pose_callback(initial_pose());self.assertIsNotNone(f.map_to_camera)
        result=f.reset_callback(Trigger.Request(),Trigger.Response())
        self.assertTrue(result.success)
        self.assertIsNone(f.map_to_camera)
        self.assertEqual(len(f.imu_samples),0)

    def test_motion_newer_than_latest_tf_prevents_initialization(self):
        f=harness()
        f.imu_samples.append(SimpleNamespace(stamp_ns=10_100_000_000,frame='imu',
            acceleration=f.imu_samples[-1].acceleration.copy(),angular_velocity=np.array([0.,0.,.3])))
        f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)

    def test_invalid_new_imu_invalidates_older_stationary_window(self):
        f=harness()
        msg=Imu();msg.header.frame_id='imu';msg.header.stamp.sec=10
        msg.header.stamp.nanosec=100_000_000;msg.linear_acceleration.x=float('nan')
        f.imu_callback(msg);f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)
        # A single subsequent good sample is not a recovered stationary window.
        msg.linear_acceleration.x=0.;msg.linear_acceleration.z=9.81
        msg.header.stamp.nanosec=90_000_000
        f.imu_callback(msg);f.initial_pose_callback(initial_pose())
        self.assertIsNone(f.map_to_camera)


if __name__=='__main__':unittest.main()
