import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('simulation', True)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self.simulation = self.get_parameter(
            'simulation'
        ).value

        self.serial_port = self.get_parameter(
            'serial_port'
        ).value

        self.baud_rate = self.get_parameter(
            'baud_rate'
        ).value

        self.serial_connection = None

        if not self.simulation:
            import serial

            self.serial_connection = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1
            )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Monitor incoming Teensy responses
        if not self.simulation:
            self.create_timer(0.05, self.read_serial_callback)

        mode = 'SIMULATION' if self.simulation else 'HARDWARE'

        self.get_logger().info(
            f'Serial Bridge started | Mode: {mode}'
        )

    def cmd_vel_callback(self, msg):

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        data = f'{vx:.3f},{vy:.3f},{wz:.3f}\n'

        if self.simulation:
            self.get_logger().info(
                f'SERIAL TX: {data.strip()}'
            )
        else:
            try:
                self.serial_connection.write(
                    data.encode('utf-8')
                )
            except Exception as error:
                self.get_logger().error(
                    f'Serial write failed: {error}'
                )

    def read_serial_callback(self):
        if self.serial_connection is None:
            return

        try:
            while self.serial_connection.in_waiting > 0:
                response = self.serial_connection.readline()
                response = response.decode(
                    'utf-8', errors='replace'
                ).strip()

                if response:
                    self.get_logger().info(
                        f'TEENSY RX: {response}'
                    )

        except Exception as error:
            self.get_logger().error(
                f'Serial read failed: {error}'
            )

    def destroy_node(self):
        if self.serial_connection is not None:
            self.serial_connection.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SerialNode()
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
