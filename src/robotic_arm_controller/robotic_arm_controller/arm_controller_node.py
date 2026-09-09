import rclpy
from rclpy.node import Node


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self.get_logger().info(
            'Robotic Arm Controller Node started'
        )


def main(args=None):
    rclpy.init(args=args)

    node = ArmControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
