#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmMonitor(Node):

    def __init__(self):
        super().__init__('arm_monitor')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joints = {}

        self.get_logger().info('==========================================')
        self.get_logger().info(' ROBOTIC ARM LIVE JOINT MONITOR')
        self.get_logger().info(' Waiting for /joint_states...')
        self.get_logger().info('==========================================')

    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            if i >= len(msg.position):
                continue

            position = msg.position[i]

            velocity = 0.0

            if i < len(msg.velocity):
                velocity = msg.velocity[i]

            self.joints[name] = {
                'position': position,
                'velocity': velocity
            }

        self.display_joint_states()

    def display_joint_states(self):

        # Clear terminal
        print('\033[2J\033[H', end='')

        print('==============================================================')
        print('              ROBOTIC ARM LIVE MONITOR')
        print('==============================================================')

        print()
        print(f'{"JOINT":<12} {"ANGLE(rad)":>14} {"ANGLE(deg)":>14} {"VELOCITY":>14}')
        print('-' * 60)

        joint_order = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        for joint in joint_order:

            if joint not in self.joints:
                print(f'{joint:<12} {"WAITING":>14}')
                continue

            position = self.joints[joint]['position']
            velocity = self.joints[joint]['velocity']

            degrees = math.degrees(position)

            print(
                f'{joint:<12} '
                f'{position:>14.4f} '
                f'{degrees:>14.2f} '
                f'{velocity:>14.4f}'
            )

        print()
        print('==============================================================')
        print(' Press Ctrl+C to stop')
        print('==============================================================')


def main(args=None):

    rclpy.init(args=args)

    node = ArmMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
