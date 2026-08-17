#!/usr/bin/env python3
"""Ramp the gimbal pitch back and forth at a chosen rate, for lag testing.

A stepped setpoint makes the gimbal slew at its own maximum, which passes over
the scene too fast to collect many detections. This ramps the setpoint instead,
so the joint tracks it and the sweep runs at --rate degrees per second.
"""
import argparse
import math
import sys

import rclpy
from mavros_msgs.msg import GimbalManagerSetAttitude
from rclpy.node import Node

sys.path.insert(0, "/stacks/baseline/sim_bridge")
from sim_bridge.projection import quat_from_rpy

SEND_HZ = 20.0

ap = argparse.ArgumentParser()
ap.add_argument("--low", type=float, default=-80.0)
ap.add_argument("--high", type=float, default=-40.0)
ap.add_argument("--rate", type=float, default=15.0, help="degrees per second")
ap.add_argument("--seconds", type=float, default=60.0)
args, _ = ap.parse_known_args()

rclpy.init()
node = Node("gimbal_sweep")
pub = node.create_publisher(
    GimbalManagerSetAttitude,
    "/mavros/gimbal_control/manager/set_attitude", 10)

state = {"pitch": args.low, "step": args.rate / SEND_HZ}


def send():
    pitch = state["pitch"] + state["step"]
    if pitch >= args.high:
        pitch, state["step"] = args.high, -abs(state["step"])
    elif pitch <= args.low:
        pitch, state["step"] = args.low, abs(state["step"])
    state["pitch"] = pitch

    cmd = GimbalManagerSetAttitude()
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.flags = 0
    cmd.gimbal_device_id = 0
    q = quat_from_rpy(0.0, math.radians(pitch), 0.0)
    cmd.q.x, cmd.q.y, cmd.q.z, cmd.q.w = q
    cmd.angular_velocity_x = math.nan
    cmd.angular_velocity_y = math.nan
    cmd.angular_velocity_z = math.nan
    pub.publish(cmd)


node.create_timer(1.0 / SEND_HZ, send)
print(f"ramping pitch {args.low} <-> {args.high} deg at {args.rate} deg/s "
      f"for {args.seconds}s", flush=True)
end = node.get_clock().now().nanoseconds / 1e9 + args.seconds
while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < end:
    rclpy.spin_once(node, timeout_sec=0.05)
print("sweep done", flush=True)
node.destroy_node()
rclpy.try_shutdown()
