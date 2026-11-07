import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusNode(Node):

    def __init__(self):
        super().__init__('status_node')

        self.robot_status = 'IDLE'

        self.create_subscription(
            String,
            '/fsm/state',
            self.state_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/robot/status',
            10
        )

        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Status Node started')

    def state_callback(self, msg):
        self.robot_status = msg.data

    def timer_callback(self):
        msg = String()
        msg.data = self.robot_status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()

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
