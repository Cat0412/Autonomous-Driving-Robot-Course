#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 마커 감지 노드
- /camera/color/image_raw 구독 → ArUco 마커 인식
- /aruco_detected (Bool) 퍼블리시 → 마커 발견 여부
- /aruco_pose (PoseStamped) 퍼블리시 → 마커 위치 (x=좌우 오프셋, z=전방 거리)
- /aruco_result_image (Image) 퍼블리시 → 디버깅용 결과 이미지
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---- ROS 파라미터 선언 ----
        self.declare_parameter('marker_length', 0.05)       # 마커 한 변 길이 (m)
        self.declare_parameter('target_marker_id', -1)       # 인식할 마커 ID (-1이면 모든 마커)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')

        # 카메라 내부 파라미터 (Orbbec 카메라 640x480 기본값)
        self.declare_parameter('fx', 605.0)
        self.declare_parameter('fy', 605.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # 파라미터 읽기
        self.marker_length = self.get_parameter('marker_length').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        camera_topic = self.get_parameter('camera_topic').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        # 카메라 매트릭스 구성
        self.camera_matrix = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ---- ArUco 설정 ----
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # solvePnP용 마커 3D 좌표 (마커 중심을 원점으로)
        ms = self.marker_length / 2.0
        self.marker_points = np.array([
            [-ms,  ms, 0],
            [ ms,  ms, 0],
            [ ms, -ms, 0],
            [-ms, -ms, 0],
        ], dtype=np.float64)

        # ---- CV Bridge ----
        self.bridge = CvBridge()

        # ---- Subscribers ----
        self.img_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # ---- Publishers ----
        self.detected_pub = self.create_publisher(Bool, '/aruco_detected', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.result_img_pub = self.create_publisher(Image, '/aruco_result_image', 10)

        # 마커 미감지 시에도 주기적으로 False를 발행하기 위한 타이머 (10Hz)
        self.marker_found = False
        self.no_detect_timer = self.create_timer(0.1, self.no_detect_callback)

        self.get_logger().info(
            f"ArUco Detector 시작 | marker_length={self.marker_length}m, "
            f"target_id={self.target_marker_id}, topic={camera_topic}"
        )

    def no_detect_callback(self):
        """마커가 감지되지 않을 때 주기적으로 False를 발행"""
        if not self.marker_found:
            msg = Bool()
            msg.data = False
            self.detected_pub.publish(msg)

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge 변환 오류: {e}")
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # ArUco 마커 검출
        corners, ids, _ = self.detector.detectMarkers(gray)

        detected = False
        best_tvec = None
        best_id = -1

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])

                # 특정 마커 ID만 인식하도록 필터링 (-1이면 모든 마커)
                if self.target_marker_id >= 0 and marker_id != self.target_marker_id:
                    continue

                current_corners = corners[i].reshape((4, 2))

                success, rvec, tvec = cv2.solvePnP(
                    self.marker_points,
                    current_corners,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if not success:
                    continue

                detected = True
                pos = tvec.reshape(-1)

                # 가장 가까운 마커를 선택
                if best_tvec is None or abs(pos[2]) < abs(best_tvec[2]):
                    best_tvec = pos
                    best_id = marker_id

                # 좌표축 그리기 (디버깅용)
                cv2.drawFrameAxes(
                    img, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_length * 0.5
                )

                # 텍스트 표시
                c = current_corners[0]
                dist = float(np.linalg.norm(pos))
                text = f"ID:{marker_id} X:{pos[0]:.2f} Z:{pos[2]:.2f}m d:{dist:.2f}m"
                cv2.putText(img, text, (int(c[0]), int(c[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ---- /aruco_detected 발행 ----
        det_msg = Bool()
        det_msg.data = detected
        self.detected_pub.publish(det_msg)
        self.marker_found = detected

        # ---- /aruco_pose 발행 ----
        if detected and best_tvec is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_link'

            # MarkerDockingController가 기대하는 형식:
            # position.x = 좌우 오프셋 (lateral)
            # position.z = 전방 거리 (distance)
            pose_msg.pose.position.x = float(best_tvec[0])  # lateral offset
            pose_msg.pose.position.y = float(best_tvec[1])  # 높이 (미사용)
            pose_msg.pose.position.z = float(best_tvec[2])  # 전방 거리

            self.pose_pub.publish(pose_msg)

            self.get_logger().info(
                f"[Detected] ID:{best_id} | "
                f"lateral={best_tvec[0]:.3f}m, distance={best_tvec[2]:.3f}m"
            )

        # ---- 디버깅용 결과 이미지 발행 ----
        try:
            out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out_msg.header = msg.header
            self.result_img_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"결과 이미지 발행 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
