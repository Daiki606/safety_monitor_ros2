import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

from ros2_safety_monitor.safety_logic import get_safety_status


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.sub_position = self.create_subscription(
            Point,
            '/robot_position',
            self.position_callback,
            10
        )

        self.pub_status = self.create_publisher(
            String,
            '/safety_status',
            10
        )

        self.get_logger().info("Safety Monitor Node started.")

    def position_callback(self, msg: Point):
        status = get_safety_status(msg.x, msg.y)

        out = String()
        out.data = status
        self.pub_status.publish(out)

        self.get_logger().info(
            f"Position: ({msg.x:.2f}, {msg.y:.2f}) -> Status: {status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
