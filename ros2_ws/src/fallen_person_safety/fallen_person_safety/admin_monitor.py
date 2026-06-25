#!/usr/bin/env python3

import json
import os
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


class AdminMonitor(Node):
    def __init__(self):
        super().__init__('fallen_person_admin_monitor')
        self.declare_parameter(
            'save_directory',
            '~/fallen_person_admin_received',
        )
        self.save_directory = Path(
            os.path.expanduser(
                str(self.get_parameter('save_directory').value)
            )
        )
        self.save_directory.mkdir(parents=True, exist_ok=True)

        alert_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/safety/incident', self.event_callback, alert_qos
        )
        self.create_subscription(
            Bool, '/safety/alarm', self.alarm_callback, alert_qos
        )
        self.create_subscription(
            CompressedImage,
            '/safety/snapshot/compressed',
            self.snapshot_callback,
            alert_qos,
        )
        self.get_logger().info(
            '관리자 모니터 시작: /safety/incident 및 사진 수신 대기'
        )

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
            text = event.get('message', msg.data)
            self.get_logger().error(f'[관리자 경고] {text}')
            self.get_logger().error(
                json.dumps(event, ensure_ascii=False, indent=2)
            )
        except json.JSONDecodeError:
            self.get_logger().error(f'[관리자 경고] {msg.data}')

    def alarm_callback(self, msg):
        if msg.data:
            self.get_logger().error(
                '경보 활성화: 사고 발생 / 작업자 쓰러짐'
            )

    def snapshot_callback(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error('수신한 사고 사진 디코딩 실패')
            return
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = self.save_directory / f'incident_{timestamp}.jpg'
        if cv2.imwrite(str(path), image):
            self.get_logger().error(f'사고 사진 저장: {path}')
        else:
            self.get_logger().error(f'사고 사진 저장 실패: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = AdminMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
