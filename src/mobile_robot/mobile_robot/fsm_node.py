import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_node')

        self.state = 'IDLE'
        self.detected = 'N'
        self.ball_center = None

        self.last_detection_time = None
        self.last_center_time = None
        self.data_timeout = 0.5

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/fsm/state', 10
        )

        self.position_pub = self.create_publisher(
            String, '/fsm/ball_position', 10
        )

        self.center_pub = self.create_publisher(
            Point, '/fsm/ball_center', 10
        )

        # Subscribers
        self.create_subscription(
            String,
            '/detection',
            self.detection_callback,
            10
        )

        self.create_subscription(
            Point,
            '/ball/center',
            self.center_callback,
            10
        )

        # Services
        self.create_service(
            Trigger, '/fsm/start', self.start_callback
        )

        self.create_service(
            Trigger, '/fsm/stop', self.stop_callback
        )

        # 10 Hz
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'FSM V1.3 started | State: IDLE'
        )

    def detection_callback(self, msg):

        if msg.data not in ('L', 'C', 'R', 'N'):
            return

        self.last_detection_time = (
            self.get_clock().now()
        )

        if msg.data != self.detected:
            self.get_logger().info(
                f'Ball Detection: {msg.data}'
            )

        self.detected = msg.data

        if self.detected == 'N':
            self.ball_center = None
            self.last_center_time = None

    def center_callback(self, msg):

        self.ball_center = msg
        self.last_center_time = (
            self.get_clock().now()
        )

    def start_callback(self, request, response):

        if self.state == 'RUNNING':
            response.success = False
            response.message = 'Robot already RUNNING'

        else:
            self.state = 'RUNNING'
            response.success = True
            response.message = 'Robot STARTED'

        return response

    def stop_callback(self, request, response):

        self.state = 'STOPPED'

        response.success = True
        response.message = 'Robot STOPPED'

        return response

    def timer_callback(self):

        now = self.get_clock().now()

        # Publish FSM state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Default: movement not permitted
        position_msg = String()
        position_msg.data = 'N'

        valid = (
            self.state == 'RUNNING'
            and self.detected in ('L', 'C', 'R')
            and self.ball_center is not None
            and self.last_detection_time is not None
            and self.last_center_time is not None
        )

        if valid:
            detection_age = (
                now - self.last_detection_time
            ).nanoseconds / 1e9

            center_age = (
                now - self.last_center_time
            ).nanoseconds / 1e9

            valid = (
                detection_age < self.data_timeout
                and center_age < self.data_timeout
            )

        if valid:

            position_msg.data = self.detected

            self.center_pub.publish(
                self.ball_center
            )

        self.position_pub.publish(position_msg)


def main(args=None):

    rclpy.init(args=args)
    node = FSMNode()

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
