import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            String, '/detection', 10
        )

        self.center_pub = self.create_publisher(
            Point, '/ball/center', 10
        )

        self.get_logger().info(
            'Blue Ball Detector started (L/C/R/N)'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green colour threshold
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([130, 255, 255])

        mask = cv2.inRange(
            hsv, lower_blue, upper_blue
        )

        # Reduce noise
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((5, 5), np.uint8)
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        region = 'N'
        ball_center = None

        # Find largest suitable green contour
        if contours:

            contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            circularity = (
                4 * np.pi * area / (perimeter ** 2)
                if perimeter > 0 else 0
            )

            if area > 500 and circularity > 0.6:

                (cx, cy), radius = cv2.minEnclosingCircle(
                    contour
                )

                cx = int(cx)
                cy = int(cy)

                ball_center = (cx, cy)

                # Divide image into 3 regions
                if cx < width / 3:
                    region = 'L'

                elif cx < 2 * width / 3:
                    region = 'C'

                else:
                    region = 'R'

        # Draw detection zones
        cv2.line(frame, (width // 3, 0),
                 (width // 3, height), (255, 255, 255), 2)

        cv2.line(frame, (2 * width // 3, 0),
                 (2 * width // 3, height), (255, 255, 255), 2)

        # Zone labels
        for label, x in [('L', width // 6),
                         ('C', width // 2),
                         ('R', 5 * width // 6)]:
            cv2.putText(frame, label, (x - 15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 2)

        # Draw detected ball
        if ball_center is not None:
            cv2.circle(frame, ball_center, int(radius),
                       (0, 0, 255), 2)

            cv2.circle(frame, ball_center, 5,
                       (0, 0, 255), -1)

            cv2.putText(frame, str(ball_center),
                        (ball_center[0] + 10,
                         ball_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)

        # Detection status
        cv2.putText(frame, f'Detection: {region}',
                    (20, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2)

        cv2.imshow('PSA ROBUST - Blue Ball Detection', frame)
        cv2.waitKey(1)

        # Publish region
        detection_msg = String()
        detection_msg.data = region

        self.detection_pub.publish(detection_msg)

        # Publish ball center if detected
        if ball_center is not None:

            center_msg = Point()
            center_msg.x = float(ball_center[0])
            center_msg.y = float(ball_center[1])
            center_msg.z = 0.0

            self.center_pub.publish(center_msg)


def main(args=None):

    rclpy.init(args=args)

    node = DetectorNode()

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
