import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


class ExamplePositionPublisher(Node):
    """
    A simple demo node that publishes a moving robot position.
    This node is only for launch/demo purposes.
    """

    def __init__(self):
        super().__init__('example_position_publisher')
        self.publisher = self.create_publisher(Point, '/robot_position', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.theta = 0.0

    def timer_callback(self):
        msg = Point()
        # Robot moves in a circle for demo purposes
        msg.x = 5.0 * math.cos(self.theta)
        msg.y = 5.0 * math.sin(self.theta)
        msg.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing position: ({msg.x:.2f}, {msg.y:.2f})')

        self.theta += 0.3


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
