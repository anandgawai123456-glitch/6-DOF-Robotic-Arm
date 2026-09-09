from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_share = FindPackageShare("robotic_arm_description")

    urdf_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "robotic_arm.urdf"
    ])

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": open(
                        "/home/anand/robotic_arm_ws/src/robotic_arm_description/urdf/robotic_arm.urdf"
                    ).read()
                }
            ]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        ),
    ])
