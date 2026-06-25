#!/usr/bin/env python3

import json
import os
from datetime import datetime
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import YOLO


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FallenPersonSafety(Node):
    """Rotate, find a fallen person, approach, notify, and beep."""

    SEARCH = 'SEARCH'
    DETECTED = 'DETECTED'
    APPROACH = 'APPROACH'
    ARRIVED = 'ARRIVED'

    def __init__(self):
        super().__init__('fallen_person_safety')

        self.declare_parameter('model_path', '')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safety')
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('inference_image_size', 640)
        self.declare_parameter('inference_rate_hz', 8.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('search_angular_speed', 0.15)
        self.declare_parameter('detection_stop_sec', 0.30)
        self.declare_parameter('detection_hold_sec', 2.0)
        self.declare_parameter('align_angular_gain', 0.8)
        self.declare_parameter('max_angular_speed', 0.15)
        self.declare_parameter('center_tolerance_ratio', 0.10)
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('stop_distance_m', 0.50)
        self.declare_parameter('minimum_valid_depth_m', 0.20)
        self.declare_parameter('maximum_valid_depth_m', 8.0)
        self.declare_parameter(
            'snapshot_directory',
            '~/fallen_person_incidents',
        )
        self.declare_parameter('local_terminal_beep', True)

        p = lambda name: self.get_parameter(name).value
        self.model_path = str(p('model_path'))
        self.confidence_threshold = float(p('confidence_threshold'))
        self.inference_image_size = int(p('inference_image_size'))
        self.search_angular_speed = float(p('search_angular_speed'))
        self.detection_stop_sec = float(p('detection_stop_sec'))
        self.detection_hold = Duration(
            seconds=float(p('detection_hold_sec'))
        )
        self.align_angular_gain = float(p('align_angular_gain'))
        self.max_angular_speed = float(p('max_angular_speed'))
        self.center_tolerance_ratio = float(p('center_tolerance_ratio'))
        self.approach_speed = float(p('approach_linear_speed'))
        self.stop_distance_m = float(p('stop_distance_m'))
        self.minimum_valid_depth_m = float(p('minimum_valid_depth_m'))
        self.maximum_valid_depth_m = float(p('maximum_valid_depth_m'))
        self.local_terminal_beep = bool(p('local_terminal_beep'))
        self.snapshot_directory = Path(
            os.path.expanduser(str(p('snapshot_directory')))
        )
        self.snapshot_directory.mkdir(parents=True, exist_ok=True)

        if not self.model_path or not Path(self.model_path).is_file():
            raise FileNotFoundError(f'YOLO model not found: {self.model_path}')

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.get_logger().info(f'YOLO classes: {self.model.names}')

        self.bridge = CvBridge()
        self.frame_lock = Lock()
        self.latest_rgb = None
        self.latest_depth = None
        self.detection = None
        self.detection_stamp = None
        self.annotated_frame = None
        self.last_distance = None

        self.state = self.SEARCH
        self.state_entered = self.get_clock().now()
        self.arrival_reported = False

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(
            Twist, str(p('cmd_vel_topic')), 10
        )
        self.event_pub = self.create_publisher(
            String, '/safety/incident', latched_qos
        )
        self.snapshot_pub = self.create_publisher(
            CompressedImage, '/safety/snapshot/compressed', latched_qos
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/safety/debug_image', 1
        )
        self.state_pub = self.create_publisher(
            String, '/safety/state', latched_qos
        )
        self.emergency_pub = self.create_publisher(
            String, '/emergency', 10
        )

        self.create_subscription(
            Image,
            str(p('rgb_topic')),
            self.rgb_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(p('depth_topic')),
            self.depth_callback,
            qos_profile_sensor_data,
        )

        inference_period = 1.0 / max(float(p('inference_rate_hz')), 0.1)
        control_period = 1.0 / max(float(p('control_rate_hz')), 1.0)
        self.create_timer(inference_period, self.run_inference)
        self.create_timer(control_period, self.control_loop)

        release = String()
        release.data = 'release'
        self.emergency_pub.publish(release)
        self.publish_state()
        self.get_logger().info(
            'Fallen-person controller ready: SEARCH -> APPROACH -> ARRIVED'
        )

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_rgb = frame
        except Exception as exc:
            self.get_logger().error(f'RGB conversion failed: {exc}')

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            with self.frame_lock:
                self.latest_depth = depth
        except Exception as exc:
            self.get_logger().error(f'Depth conversion failed: {exc}')

    def run_inference(self):
        with self.frame_lock:
            if self.latest_rgb is None:
                return
            frame = self.latest_rgb.copy()

        try:
            result = self.model.predict(
                source=frame,
                conf=self.confidence_threshold,
                imgsz=self.inference_image_size,
                device='cpu',
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        best = None
        if result.boxes is not None:
            for box in result.boxes:
                confidence = float(box.conf[0].detach().cpu())
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(int)
                candidate = {
                    'confidence': confidence,
                    'box': tuple(int(value) for value in xyxy),
                }
                if best is None or confidence > best['confidence']:
                    best = candidate

        if best is not None:
            self.detection = best
            self.detection_stamp = self.get_clock().now()
            if self.state == self.SEARCH:
                self.publish_stop()
                self.transition(self.DETECTED)
                self.publish_event(
                    'fallen_person_detected',
                    '작업자 쓰러짐 감지 / 접근 시작',
                    confidence=round(best['confidence'], 3),
                )

        annotated = frame.copy()
        if self.detection_is_recent():
            x1, y1, x2, y2 = self.detection['box']
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(
                annotated,
                f'FALLEN PERSON {self.detection["confidence"]:.2f}',
                (max(0, x1), max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        status = f'STATE: {self.state}'
        if self.last_distance is not None:
            status += f'  DIST: {self.last_distance:.2f}m'
        cv2.putText(
            annotated,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
        self.annotated_frame = annotated
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        )

    def control_loop(self):
        if self.state == self.SEARCH:
            cmd = Twist()
            cmd.angular.z = self.search_angular_speed
            self.cmd_pub.publish(cmd)
            return

        if self.state == self.DETECTED:
            self.publish_stop()
            elapsed = (
                self.get_clock().now() - self.state_entered
            ).nanoseconds / 1e9
            if elapsed >= self.detection_stop_sec:
                self.transition(self.APPROACH)
            return

        if self.state == self.APPROACH:
            if not self.detection_is_recent():
                self.get_logger().warning(
                    'Person lost for 2 seconds; returning to rotation search'
                )
                self.last_distance = None
                self.transition(self.SEARCH)
                return

            horizontal_error = self.horizontal_error()
            distance = self.depth_distance_for_detection()
            self.last_distance = distance

            if distance is not None and distance <= self.stop_distance_m:
                self.publish_stop()
                self.transition(self.ARRIVED)
                self.report_arrival(distance)
                return

            cmd = Twist()
            cmd.angular.z = clamp(
                -horizontal_error * self.align_angular_gain,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

            # First turn until the person is near the image centre, then
            # approach while making small steering corrections.
            if abs(horizontal_error) <= self.center_tolerance_ratio:
                cmd.linear.x = self.approach_speed

            self.cmd_pub.publish(cmd)
            return

        if self.state == self.ARRIVED:
            self.publish_stop()

    def detection_is_recent(self):
        if self.detection is None or self.detection_stamp is None:
            return False
        return (
            self.get_clock().now() - self.detection_stamp
        ) <= self.detection_hold

    def horizontal_error(self):
        if self.detection is None or self.latest_rgb is None:
            return 0.0
        x1, _, x2, _ = self.detection['box']
        centre_x = 0.5 * (x1 + x2)
        image_width = float(self.latest_rgb.shape[1])
        return (centre_x - 0.5 * image_width) / (0.5 * image_width)

    def depth_distance_for_detection(self):
        if (
            self.latest_depth is None
            or self.latest_rgb is None
            or self.detection is None
        ):
            return None

        depth = self.latest_depth
        rgb_h, rgb_w = self.latest_rgb.shape[:2]
        depth_h, depth_w = depth.shape[:2]
        x1, y1, x2, y2 = self.detection['box']

        # Centre of the YOLO box only. Using the lower edge previously picked
        # up the floor and incorrectly reported 0.3 m for a distant person.
        rx1 = int((x1 + 0.35 * (x2 - x1)) * depth_w / rgb_w)
        rx2 = int((x1 + 0.65 * (x2 - x1)) * depth_w / rgb_w)
        ry1 = int((y1 + 0.30 * (y2 - y1)) * depth_h / rgb_h)
        ry2 = int((y1 + 0.70 * (y2 - y1)) * depth_h / rgb_h)
        rx1 = int(clamp(rx1, 0, depth_w - 1))
        rx2 = int(clamp(rx2, rx1 + 1, depth_w))
        ry1 = int(clamp(ry1, 0, depth_h - 1))
        ry2 = int(clamp(ry2, ry1 + 1, depth_h))

        values = depth[ry1:ry2, rx1:rx2].astype(np.float32)
        if np.issubdtype(depth.dtype, np.integer):
            values *= 0.001
        values = values[np.isfinite(values)]
        values = values[
            (values >= self.minimum_valid_depth_m)
            & (values <= self.maximum_valid_depth_m)
        ]
        if values.size < 20:
            return None
        return float(np.median(values))

    def transition(self, new_state):
        old_state = self.state
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.get_logger().warning(f'State: {old_state} -> {new_state}')
        self.publish_state()

    def publish_state(self):
        message = String()
        message.data = self.state
        self.state_pub.publish(message)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_event(self, event_type, message_text, **extra):
        message = String()
        payload = {
            'type': event_type,
            'message': message_text,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
        }
        payload.update(extra)
        message.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(message)

    def report_arrival(self, distance):
        if self.arrival_reported:
            return
        self.arrival_reported = True

        frame = (
            self.annotated_frame.copy()
            if self.annotated_frame is not None
            else self.latest_rgb.copy()
        )
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_path = self.snapshot_directory / f'incident_{timestamp}.jpg'
        cv2.imwrite(str(image_path), frame)

        compressed = self.bridge.cv2_to_compressed_imgmsg(
            frame, dst_format='jpg'
        )
        compressed.header.stamp = self.get_clock().now().to_msg()
        compressed.header.frame_id = 'camera_color_optical_frame'
        self.snapshot_pub.publish(compressed)

        self.publish_event(
            'arrived_at_fallen_person',
            '작업자 쓰러짐 / 로봇 도착',
            distance_m=round(float(distance), 3),
            snapshot_path=str(image_path),
        )
        if self.local_terminal_beep:
            print('\a삐!\a삐!', flush=True)
        self.get_logger().error(
            f'ARRIVED: distance={distance:.2f}m, snapshot={image_path}'
        )

    def destroy_node(self):
        for _ in range(5):
            self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FallenPersonSafety()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
