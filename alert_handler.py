import rclpy
from rclpy.node import Node


class AlertHandler(Node):
    def __init__(self):
        super().__init__('alert_handler')
        self.get_logger().info('Alert handler is initialized')
        self.alert_sub = self.create_subscription(
            "/alert_event", String, self.alert_callback, 10)


def main(args=None):
    rclpy.init(args=args)

    node = AlertHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
