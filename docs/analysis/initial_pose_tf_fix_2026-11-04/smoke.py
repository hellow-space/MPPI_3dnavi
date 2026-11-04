#!/usr/bin/env python3
"""Isolated synthetic ROS transport check; no velocity publisher or Nav2."""
import importlib.util
import math
import os
from pathlib import Path
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '143' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Require ROS_DOMAIN_ID=143 ROS_LOCALHOST_ONLY=1; never use the robot domain')

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, StaticTransformBroadcaster, TransformListener

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('smoke_initial_pose',ROOT/'scripts/initial_pose_tf.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rclpy.init(args=['--ros-args','-p','imu_topic:=/review/imu','-p','map_frame:=review_map',
                '-p','odom_frame:=review_camera','-p','base_frame:=review_base'])
source=Node('initial_pose_review_source')
node=module.InitialPoseTF()
executor=SingleThreadedExecutor();executor.add_node(source);executor.add_node(node)
imu_pub=source.create_publisher(Imu,'/review/imu',qos_profile_sensor_data)
pose_pub=source.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
broadcaster=TransformBroadcaster(source);static=StaticTransformBroadcaster(source)
buffer=Buffer();listener=TransformListener(buffer,source)
t=TransformStamped();t.header.frame_id='review_base';t.child_frame_id='review_imu'
t.transform.rotation.w=1.;static.sendTransform(t)
state={'body_pitch':10.,'imu_age':0.}


def tick():
    now=source.get_clock().now()
    body=math.radians(state['body_pitch'])
    camera_angle=math.radians(state['body_pitch']-34.)
    tf=TransformStamped();tf.header.stamp=now.to_msg()
    tf.header.frame_id='review_camera';tf.child_frame_id='review_base'
    tf.transform.translation.z=.5
    tf.transform.rotation.y=math.sin(camera_angle/2);tf.transform.rotation.w=math.cos(camera_angle/2)
    broadcaster.sendTransform(tf)
    imu=Imu();imu.header.frame_id='review_imu'
    imu.header.stamp=Time(nanoseconds=now.nanoseconds-int(state['imu_age']*1e9)).to_msg()
    imu.linear_acceleration.x=-9.81*math.sin(body);imu.linear_acceleration.z=9.81*math.cos(body)
    imu_pub.publish(imu)


timer=source.create_timer(.01,tick)


def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:executor.spin_once(timeout_sec=.01)


def send_pose():
    p=PoseWithCovarianceStamped();p.header.frame_id='review_map'
    p.header.stamp=source.get_clock().now().to_msg()
    p.pose.pose.position.x=3.;p.pose.pose.position.y=4.
    p.pose.pose.orientation.z=math.sin(math.radians(35)/2)
    p.pose.pose.orientation.w=math.cos(math.radians(35)/2)
    pose_pub.publish(p)


def measured_pose():
    tf=buffer.lookup_transform('review_map','review_base',Time())
    r=module.tf_to_matrix(tf)
    pitch=math.degrees(math.asin(-np.clip(r[2,0],-1.,1.)))
    yaw=math.degrees(math.atan2(r[1,0],r[0,0]))
    return r,pitch,yaw


try:
    spin(2.)
    send_pose();spin(.5)
    assert node.map_to_camera is not None,'gravity calibration was not accepted'
    first=node.map_to_camera.copy()
    result,pitch,yaw=measured_pose()
    np.testing.assert_allclose(result[:3,3],[3.,4.,0.],atol=1e-6)
    assert abs(pitch-10.)<1e-6 and abs(yaw-35.)<1e-6,(pitch,yaw)
    print('PASS: ROS IMU/TF/initialpose -> gravity alignment -> published map TF; body pitch retained',flush=True)
    state['body_pitch']=15.;spin(.6)
    _,pitch,yaw=measured_pose()
    np.testing.assert_array_equal(node.map_to_camera,first)
    assert abs(pitch-15.)<1e-6
    print('PASS: body pitch changes propagate without redefining map up',flush=True)
    client=source.create_client(Trigger,'/initial_pose_tf/reset')
    assert client.wait_for_service(timeout_sec=2.)
    future=client.call_async(Trigger.Request())
    deadline=time.monotonic()+2.
    while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
    assert future.done() and future.result().success and node.map_to_camera is None
    print('PASS: reset service invalidates initialization',flush=True)
    state['imu_age']=2.;spin(1.3)
    send_pose();spin(.3)
    assert node.map_to_camera is None and 'raw IMU age' in node.imu_rejection_reason
    print('PASS: stale raw IMU prevents calibration despite fresh pose TF',flush=True)
    state['imu_age']=0.;spin(1.5)
    send_pose();spin(.4)
    assert node.map_to_camera is not None
    print('PASS: fresh stationary data restores ability to initialize',flush=True)
finally:
    executor.shutdown();node.destroy_node();source.destroy_node();rclpy.shutdown()
