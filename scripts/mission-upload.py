#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import rclpy
from mavros_msgs.srv import WaypointPush, WaypointSetCurrent
from umd_uas.umd_uas_mission import UASMissionNode
from rclpy.node import Node


class MissionUploader(Node):
    def __init__(self, vehicle: int):
        super().__init__("px4sim_mission_uploader")
        self.client = self.create_client(WaypointPush, f"/uas{vehicle}/mission/push")
        self.current_client = self.create_client(
            WaypointSetCurrent, f"/uas{vehicle}/mission/set_current"
        )

    def upload(self, plan: Path):
        waypoints = UASMissionNode.load_plan_waypoints(str(plan))
        if not self._wait_for_service():
            raise RuntimeError(f"MAVROS mission/push did not appear: {self.client.srv_name}")
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints
        future = self.client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = future.result()
        if not result.success:
            raise RuntimeError(
                f"FCU rejected mission ({result.wp_transfered}/{len(waypoints)} items)"
            )
        transferred = result.wp_transfered
        if not self._wait_for_client(self.current_client):
            raise RuntimeError(
                f"MAVROS mission/set_current did not appear: {self.current_client.srv_name}"
            )
        current = WaypointSetCurrent.Request()
        current.wp_seq = 0
        future = self.current_client.call_async(current)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = future.result()
        if not result.success:
            raise RuntimeError("FCU rejected mission cursor reset")
        return len(waypoints), transferred

    def _wait_for_service(self):
        return self._wait_for_client(self.client)

    def _wait_for_client(self, client):
        for _ in range(30):
            if client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=1.0)
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vehicle", type=int)
    parser.add_argument("plan", type=Path)
    args = parser.parse_args()
    if args.plan.suffix != ".plan" or not args.plan.is_file():
        raise RuntimeError(f"not a readable QGroundControl plan: {args.plan}")
    with args.plan.open(encoding="utf-8") as stream:
        if json.load(stream).get("fileType") != "Plan":
            raise RuntimeError(f"not a QGroundControl plan: {args.plan}")
    rclpy.init()
    node = MissionUploader(args.vehicle)
    uploaded, total = node.upload(args.plan)
    print(f"uploaded {total}/{uploaded} mission items")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(str(exc))
