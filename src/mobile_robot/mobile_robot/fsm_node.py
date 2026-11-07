import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_node')

        self.state = 'IDLE'
        self.detected = 'N'

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/fsm/state', 10
        )

        self.position_pub = self.create_publisher(
            String, '/fsm/ball_position', 10
        )

        # Subscriber
        self.create_subscription(
            String, '/detection', self.detection_callback, 10
        )

        # Services
        self.create_service(
            Trigger, '/fsm/start', self.start_callback
        )

        self.create_service(
            Trigger, '/fsm/stop', self.stop_callback
        )

        # Publish at 10 Hz
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('FSM started | State: IDLE')

    def detection_callback(self, msg):
        if msg.data in ('L', 'C', 'R', 'N'):
            if msg.data != self.detected:
                self.detected = msg.data
                self.get_logger().info(
                    f'Ball Detection: {self.detected}'
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
        # Publish FSM state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Forward detection only when RUNNING
        position_msg = String()

        if self.state == 'RUNNING':
            position_msg.data = self.detected
        else:
            position_msg.data = 'N'

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
