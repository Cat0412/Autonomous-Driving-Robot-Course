#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist


def clamp(value, min_value, max_value):
    """값을 min_value와 max_value 사이로 제한"""
    return max(min(value, max_value), min_value)

class MarkerDockingController(Node):
    """
    1) /aruco_detected (Bool) 구독 → 마커 인식 여부
    2) /aruco_pose (PoseStamped) 구독 → 마커 Pose(x: lateral, z: distance)
    3) 10Hz 제어 루프
       - 항상 도킹 로직 활성화
         - Pose 정보 없음 → 제자리 회전(reacquire)
         - 마커 인식 시
           * 정렬 모드 (lateral offset > threshold) → angular 속도 제어
           * 접근 모드 (offset ≤ threshold) → linear PID 제어
         - 목표 거리(5cm) 도달 → 도킹 완료 플래그 발행 및 정지
    """

    def __init__(self):
        super().__init__('marker_docking_controller')

        # 도킹 완료 플래그
        self.docking_done = False

        # 마커 감지 정보
        self.marker_detected = False
        self.latest_lateral = None  # x축 오프셋 (m)
        self.latest_distance = None # z축 거리 (m)

        #Subscribe
        # 1) ArUco 감지 플래그 구독
        self.flag_sub = self.create_subscription(
            Bool, '/aruco_detected', self.flag_callback, 10)

        # 2) ArUco Pose 구독
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10)

        # 3) battery subscribe
        self.battery_sub = self.create_subscription(
            Bool, '/battery', self.flag_callback, 10)

        #Publish
        # 1) cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2) 도킹 완료 플래그 퍼블리셔
        self.dock_pub = self.create_publisher(Bool, '/docking_complete', 10)


        # --- PID 거리 제어 파라미터 ---
        self.Kp_d, self.Ki_d, self.Kd_d = 1.2, 0.0, 0.3
        self.prev_error_d = 0.0
        self.integral_d   = 0.0

        # --- P yaw 제어 파라미터 ---
        self.Kp_a        = 2.0
        self.max_ang_vel = 1.0       # rad/s

        # --- 목표 및 한계값 ---
        self.target_distance = 0.20  # 목표 거리 5cm
        self.error_tolerance = 0.02  # 거리 허용 오차 ±2cm
        self.offset_tolerance = 0.02 # 정렬 허용 오프셋 ±2cm
        self.max_lin_vel    = 0.3    # 최대 선속도 0.3 m/s
        self.search_ang_vel = 0.5    # 재탐색 회전 속도 (rad/s)

        # 제어 루프 타이머 (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Marker Docking Controller 시작 (토픽 및 축 매핑 업데이트)")

    def flag_callback(self, msg: Bool):
        """ArUco 감지 여부 수신"""
        self.marker_detected = msg.data
        self.get_logger().info(f"[Marker] detected = {self.marker_detected}")

    def pose_callback(self, msg: PoseStamped):
        """ArUco Pose(x, z) 수신: x=좌우 오프셋, z=전방 거리"""
        self.latest_lateral = msg.pose.position.x
        self.latest_distance = msg.pose.position.z

    def control_loop(self):
        cmd = Twist()

        # 이미 도킹 완료 상태면 동작 중지
        if self.docking_done:
            return

        # Pose 정보 없음 → 재탐색 회전
        if self.latest_lateral is None or self.latest_distance is None:
            cmd.angular.z = self.search_ang_vel
            self.cmd_pub.publish(cmd)
            return

        # 마커 감지 여부에 따라 로직 분기
        if self.marker_detected:
            # 1) 정렬 모드: 좌우 오프셋 기준 angular 제어
            if abs(self.latest_lateral) > self.offset_tolerance:
                cmd.linear.x  = 0.0
                # 오프셋 <0 → 왼쪽, >0 → 오른쪽
                ang_error = -self.latest_lateral
                cmd.angular.z = clamp(ang_error * self.Kp_a,
                                      -self.max_ang_vel, self.max_ang_vel)
            else:
                # 2) 접근 모드: 선속도 PID 제어, angular 정지
                cmd.angular.z = 0.0
                err_d = self.latest_distance - self.target_distance
                self.integral_d += err_d
                deriv_d = err_d - self.prev_error_d
                self.prev_error_d = err_d

                lin = (self.Kp_d * err_d +
                       self.Ki_d * self.integral_d +
                       self.Kd_d * deriv_d)
                cmd.linear.x = clamp(lin, -self.max_lin_vel, self.max_lin_vel)

                # 목표 거리 도달 → 도킹 완료
                if abs(err_d) < self.error_tolerance:
                    cmd = Twist()
                    self.docking_done = True
                    done = Bool(); done.data = True
                    self.dock_pub.publish(done)
                    self.get_logger().info("[Docking] 완료 → 도킹 플래그 발행")
        else:
            # 3) 마커 미감지 → 재탐색 회전
            cmd.linear.x  = 0.0
            cmd.angular.z = self.search_ang_vel
            self.get_logger().info("[Search] 마커 미감지 → 회전 중")

        # 명령 발행
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        # 종료 시 도킹 플래그 초기화
        reset = Bool(); reset.data = False
        self.dock_pub.publish(reset)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

