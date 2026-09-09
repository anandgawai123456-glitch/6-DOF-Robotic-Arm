from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([

        # Wait for Gazebo + ros2_control controller_manager
        TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster,
            ],
        ),

        # Start trajectory controller after broadcaster
        TimerAction(
            period=5.0,
            actions=[
                joint_trajectory_controller,
            ],
        ),
    ])
