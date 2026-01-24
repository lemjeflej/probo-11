from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("franka_sonde"), 
                "urdf",
                "fr3_sur_rail.xacro"
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([

        # Publie TF à partir de l’URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description
            }],
            output="screen"
        ),

        # UNE SEULE source de /joint_states (rail_joint)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),

        # Visualisation
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
            FindPackageShare("franka_sonde"),
            "rviz",
            "fr3_sur_rail.rviz"
            ])],
            output="screen"
        )
    ])
