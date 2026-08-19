#!/usr/bin/env python3
"""WSKR web-dashboard helper.

Publishes a single latched topic — ``wskr_web/available_models`` — carrying a
JSON array of model basenames found in ``<share/wskr>/models/``. The Foxglove
dashboard reads this to show users what they can type into the
``WSKR/autopilot/model_filename`` Publish panel.

No per-frame work. One directory scan at startup, one latched publish, then
the node idles.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def _scan_models() -> List[str]:
    try:
        share = Path(get_package_share_directory('wskr'))
    except Exception:  # noqa: BLE001
        return []
    models_dir = share / 'models'
    if not models_dir.is_dir():
        return []
    return sorted(p.name for p in models_dir.glob('*.json') if p.is_file())


class WskrWebHelper(Node):
    def __init__(self) -> None:
        super().__init__('wskr_web_helper')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(String, 'wskr_web/available_models', latched)

        models = _scan_models()
        msg = String()
        msg.data = json.dumps(models)
        self._pub.publish(msg)
        self.get_logger().info(
            f'Published wskr_web/available_models ({len(models)} model(s)): {models}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WskrWebHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
