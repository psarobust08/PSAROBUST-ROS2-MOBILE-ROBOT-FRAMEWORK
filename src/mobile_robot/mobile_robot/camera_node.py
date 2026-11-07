import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(
            Image, '/camera/image_raw', 10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera /dev/video0')
            raise RuntimeError('Camera not available')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.timer = self.create_timer(
            1.0 / 30.0, self.timer_callback
        )

        self.get_logger().info('Camera Node started')

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame, encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraNode()
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
