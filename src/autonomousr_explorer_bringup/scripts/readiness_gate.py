#!/usr/bin/env python3
"""等待地图、代价地图和能力服务就绪后通知 bringup。

该进程只负责启动编排，不参与业务逻辑。它通过 ROS graph 判断节点是否已
经发布数据并提供服务，避免使用固定秒数猜测 Nav2 或 SLAM 的启动时间。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


class ReadinessGate(Node):
    """按要求检查 ROS topic 发布者和 service 是否出现在 graph 中。"""

    def __init__(self, required_topics, required_services):
        super().__init__("bringup_readiness_gate")
        self._required_topics = tuple(required_topics)
        self._required_services = tuple(required_services)

    def missing_items(self):
        """返回当前尚未就绪的 topic/service，便于超时日志定位。"""
        topic_names = {
            name for name, _ in self.get_topic_names_and_types(no_demangle=True)
        }
        service_names = {
            name for name, _ in self.get_service_names_and_types(no_demangle=True)
        }

        missing_topics = [
            topic for topic in self._required_topics
            if topic not in topic_names or self.count_publishers(topic) == 0
        ]
        missing_services = [
            service for service in self._required_services
            if service not in service_names
        ]
        return missing_topics, missing_services


def parse_args():
    parser = argparse.ArgumentParser(description="ROS bringup readiness gate")
    parser.add_argument("--topic", dest="topics", action="append", default=[])
    parser.add_argument("--service", dest="services", action="append", default=[])
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--poll-period-sec", type=float, default=0.2)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.timeout_sec <= 0.0 or args.poll_period_sec <= 0.0:
        print("READINESS_GATE_INVALID_ARGUMENT", flush=True)
        return 2

    rclpy.init()
    gate = ReadinessGate(args.topics, args.services)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok():
            rclpy.spin_once(gate, timeout_sec=args.poll_period_sec)
            missing_topics, missing_services = gate.missing_items()
            if not missing_topics and not missing_services:
                print("READINESS_GATE_READY", flush=True)
                # 保持进程存活，避免 launch 在能力节点运行期间误判 gate 已退出。
                while rclpy.ok():
                    rclpy.spin_once(gate, timeout_sec=1.0)
                return 0

            if time.monotonic() >= deadline:
                print(
                    "READINESS_GATE_TIMEOUT "
                    f"topics={missing_topics} services={missing_services}",
                    flush=True,
                )
                return 2
    finally:
        gate.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
