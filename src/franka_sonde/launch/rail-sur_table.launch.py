from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "cat ",
            PathJoinSubstitution([
                FindPackageShare("franka_sonde"),
                "urdf",
                "rail_sur_table.xacro"
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        # JointStatePublisher GUI for prismatic joint control
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
            FindPackageShare("franka_sonde"),
            "rviz",
            "trm.rviz"
            ])],
            output="screen"
        )
    ])
