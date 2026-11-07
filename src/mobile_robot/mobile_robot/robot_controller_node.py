import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist


class RobotControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        # Camera configuration
        self.image_width = 640.0
        self.image_height = 480.0

        # Proportional controller
        self.kx = 0.3
        self.ky = 0.3
        self.max_velocity = 0.3
        self.deadband = 0.05

        # Detection information
        self.ball_position = 'N'
        self.ball_center = None
        self.last_center_time = None
        self.data_timeout = 0.5

        # Subscribers
        self.create_subscription(
            String,
            '/fsm/ball_position',
            self.position_callback,
            10
        )

        self.create_subscription(
            Point,
            '/fsm/ball_center',
            self.center_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # 10 Hz control loop
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Robot Controller V1.3 started | Dynamic P-Control'
        )

    def position_callback(self, msg):

        if msg.data not in ('L', 'C', 'R', 'N'):
            self.ball_position = 'N'
            self.ball_center = None
            return

        self.ball_position = msg.data

        if msg.data == 'N':
            self.ball_center = None
            self.last_center_time = None

    def center_callback(self, msg):

        if not (
            0.0 <= msg.x < self.image_width
            and 0.0 <= msg.y < self.image_height
        ):
            self.ball_center = None
            self.last_center_time = None
            return

        self.ball_center = (msg.x, msg.y)
        self.last_center_time = self.get_clock().now()

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def timer_callback(self):

        cmd = Twist()

        valid = (
            self.ball_position in ('L', 'C', 'R')
            and self.ball_center is not None
            and self.last_center_time is not None
        )

        if valid:
            age = (
                self.get_clock().now()
                - self.last_center_time
            ).nanoseconds / 1e9

            valid = 0.0 <= age < self.data_timeout

        if valid:
            cx, cy = self.ball_center

            # Normalized image errors
            ex = (
                cx - self.image_width / 2.0
            ) / (self.image_width / 2.0)

            ey = (
                self.image_height / 2.0 - cy
            ) / (self.image_height / 2.0)

            # Deadband
            if abs(ex) < self.deadband:
                ex = 0.0

            if abs(ey) < self.deadband:
                ey = 0.0

            # Dynamic velocity calculation
            vx = self.kx * ey
            vy = -self.ky * ex

            cmd.linear.x = self.clamp(
                vx, self.max_velocity
            )

            cmd.linear.y = self.clamp(
                vy, self.max_velocity
            )

            cmd.angular.z = 0.0

        # Invalid detection / STOP / timeout:
        # Twist remains zero.

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

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
