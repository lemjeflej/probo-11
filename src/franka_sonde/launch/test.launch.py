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
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description
            }],
            output="screen"
        ),

        # Lancer le noeud de test
        # Node(
        #     package="controleurs",
        #     executable="test_node",
        #     output="screen"
        # ),
        
        # Lancer le noeud de position initiale
        Node(
            package="controleurs",
            executable="initial_pose_controller",
            output="screen"
        ),
        
        # RViz
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