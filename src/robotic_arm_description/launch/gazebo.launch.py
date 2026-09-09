import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():

    pkg_share = FindPackageShare(
        package="robotic_arm_description"
    ).find("robotic_arm_description")

    gazebo_resource_path = os.path.dirname(pkg_share)

    robot_file = os.path.join(
        pkg_share,
        "urdf",
        "robotic_arm_gazebo.xacro"
    )

    # ============================================================
    # GAZEBO RESOURCE PATH
    # ============================================================

    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gazebo_resource_path
    )

    # ============================================================
    # GAZEBO FORTRESS
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare(
                    package="ros_gz_sim"
                ).find("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items()
    )

    # ============================================================
    # ROBOT DESCRIPTION
    #
    # IMPORTANT:
    # xacro output must explicitly be treated as a STRING.
    # Otherwise launch attempts to parse robot_description as YAML.
    # ============================================================

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                robot_file
            ]),
            value_type=str
        )
    }

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description
        ]
    )

    # ============================================================
    # SPAWN ROBOT IN GAZEBO
    # ============================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "robotic_arm",
            "-allow_renaming",
            "true",
        ],
        output="screen"
    )

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

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

    # ============================================================
    # JOINT TRAJECTORY CONTROLLER
    # ============================================================

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

    # ============================================================
    # START JOINT STATE BROADCASTER
    # ============================================================

    start_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster
        ],
    )

    # ============================================================
    # START TRAJECTORY CONTROLLER
    # ============================================================

    start_joint_trajectory_controller = TimerAction(
        period=5.0,
        actions=[
            joint_trajectory_controller
        ],
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([
        set_gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
    ])