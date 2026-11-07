import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.ball_position = 'N'

        # Subscribe to FSM output
        self.create_subscription(
            String,
            '/fsm/ball_position',
            self.position_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publish at 10 Hz
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Robot Controller started')

    def position_callback(self, msg):
        if msg.data in ('L', 'C', 'R', 'N'):
            self.ball_position = msg.data

    def timer_callback(self):
        cmd = Twist()

        if self.ball_position == 'L':
            cmd.linear.y = 0.2

        elif self.ball_position == 'C':
            cmd.linear.x = 0.2

        elif self.ball_position == 'R':
            cmd.linear.y = -0.2

        # N = all velocities zero

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
