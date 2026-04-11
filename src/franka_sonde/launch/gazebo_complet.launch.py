# gazebo_complet.launch.py
# =========================
# Séquence de lancement :
#   1. Gazebo vide
#   2. robot_state_publisher  (srr.xacro → /robot_description)
#   3. spawn du robot dans Gazebo
#   4. joint_state_broadcaster activé
#   5. fr3_arm_controller activé  (JointTrajectoryController position)
#   6. rail_state_publisher démarré (publie rail_joint sur /joint_states)
#   7. cartesian_commander démarré
#      → envoie la trajectoire vers la pose "ready" au démarrage
#      → attend ensuite des poses sur /target_pose
#      → commande le rail via /target_rail_pos (std_msgs/Float64)

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, robot_type, load_gripper, rail_position):
    robot_type_str    = context.perform_substitution(robot_type)
    load_gripper_str  = context.perform_substitution(load_gripper)
    rail_position_str = context.perform_substitution(rail_position)

    srr_xacro = os.path.join(
        get_package_share_directory('franka_sonde'),
        'urdf', 'srr.xacro'
    )

    robot_description_config = xacro.process_file(
        srr_xacro,
        mappings={
            'robot_type':     robot_type_str,
            'hand':           load_gripper_str,
            'gazebo_effort':  'false',
            'rail_position':  rail_position_str,
        }
    )

    # Swap the Franka yaml path so the CM knows about fr3_arm_controller at startup.
    original_yaml = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'config', 'franka_gazebo_controllers.yaml'
    )
    our_yaml = os.path.join(
        get_package_share_directory('franka_sonde'),
        'config', 'franka_gazebo_controllers.yaml'
    )
    robot_description_xml = robot_description_config.toxml().replace(
        original_yaml, our_yaml
    )

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
    )]


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='fr3',
        description='fr3, fp3 ou fer'
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper', default_value='false',
        description='Toujours false (sonde custom)'
    )
    rail_position_arg = DeclareLaunchArgument(
        'rail_position', default_value='0.0',
        description='Position initiale du rail (m), entre 0.0 et 1.7'
    )

    robot_type    = LaunchConfiguration('robot_type')
    load_gripper  = LaunchConfiguration('load_gripper')
    rail_position = LaunchConfiguration('rail_position')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, rail_position]
    )

    # Gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join([
        os.path.dirname(get_package_share_directory('franka_description')),
        os.path.dirname(get_package_share_directory('franka_sonde')),
    ])
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'probo11'],
        output='screen',
    )

    patient_urdf = os.path.join(
        get_package_share_directory('franka_sonde'),
        'urdf', 'patient_scene.urdf'
    )
    spawn_patient = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', patient_urdf, '-name', 'patient'],
        output='screen',
    )

    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'fr3_arm_controller'],
        output='screen'
    )

    cartesian_commander = TimerAction(
        period=3.0,
        actions=[Node(
            package='controleurs',
            executable='cartesian_commander',
            name='cartesian_commander',
            output='screen',
            parameters=[{
                'move_duration':  5.0,
                'ik_max_iter':    200,
                'ik_tolerance':   1e-5,
                'rail_position':  rail_position,
            }],
        )]
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        rail_position_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_patient,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_arm_ctrl],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_ctrl,
                on_exit=[cartesian_commander],
            )
        ),
    ])
