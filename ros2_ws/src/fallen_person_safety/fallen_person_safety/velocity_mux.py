#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node


class SafetyVelocityMux(Node):
    """Give fresh safety commands priority over Nav2 velocity commands."""

    def __init__(self):
        super().__init__('safety_velocity_mux')

        self.declare_parameter('safety_topic', '/cmd_vel_safety')
        self.declare_parameter('navigation_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('output_topic', '/cmd_vel')
        # CPU YOLO inference can briefly block the Python executor. Keep the
        # latest controller command long enough that the motor does not pulse
        # between SAFETY and STOP while one frame is being inferred.
        self.declare_parameter('safety_timeout_sec', 2.0)
        self.declare_parameter('navigation_timeout_sec', 0.50)
        self.declare_parameter('publish_rate_hz', 30.0)

        p = lambda name: self.get_parameter(name).value
        self.safety_timeout = Duration(
            seconds=float(p('safety_timeout_sec'))
        )
        self.navigation_timeout = Duration(
            seconds=float(p('navigation_timeout_sec'))
        )

        self.latest_safety = None
        self.latest_safety_stamp = None
        self.latest_navigation = None
        self.latest_navigation_stamp = None
        self.selected_source = None

        self.output_pub = self.create_publisher(
            Twist, str(p('output_topic')), 10
        )
        self.create_subscription(
            Twist,
            str(p('safety_topic')),
            self.safety_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(p('navigation_topic')),
            self.navigation_callback,
            10,
        )

        rate = max(float(p('publish_rate_hz')), 1.0)
        self.create_timer(1.0 / rate, self.publish_selected_velocity)
        self.get_logger().info(
            'Velocity mux ready: safety commands have priority over Nav2'
        )

    def safety_callback(self, msg):
        self.latest_safety = msg
        self.latest_safety_stamp = self.get_clock().now()

    def navigation_callback(self, msg):
        self.latest_navigation = msg
        self.latest_navigation_stamp = self.get_clock().now()

    def publish_selected_velocity(self):
        now = self.get_clock().now()

        if (
            self.latest_safety is not None
            and self.latest_safety_stamp is not None
            and now - self.latest_safety_stamp <= self.safety_timeout
        ):
            command = self.latest_safety
            source = 'SAFETY'
        elif (
            self.latest_navigation is not None
            and self.latest_navigation_stamp is not None
            and now - self.latest_navigation_stamp <= self.navigation_timeout
        ):
            command = self.latest_navigation
            source = 'NAVIGATION'
        else:
            command = Twist()
            source = 'STOP'

        self.output_pub.publish(command)
        if source != self.selected_source:
            self.selected_source = source
            self.get_logger().info(f'Velocity source selected: {source}')

    def destroy_node(self):
        for _ in range(5):
            self.output_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyVelocityMux()
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
