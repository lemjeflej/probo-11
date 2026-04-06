# gazebo_complet.launch.py
# =========================
# Lance la simulation Gazebo du robot complet (srr.xacro).
#
# Contrôleurs chargés :
#   joint_state_broadcaster         (toujours)
#   joint_position_example_controller  (maintient la pose initiale)
#
# Le joint_position_example_controller est déjà enregistré dans
# franka_gazebo_controllers.yaml (chargé par le plugin Ignition).
# Il commande les joints en POSITION : le robot tient sa pose initiale
# avec une légère oscillation de démonstration (~4.5°).
#
# Prochain pas : remplacer joint_position_example_controller par notre
# propre plugin ros2_control qui acceptera des commandes cartésiennes.

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
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, robot_type, load_gripper):
    robot_type_str   = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)

    srr_xacro = os.path.join(
        get_package_share_directory('franka_sonde'),
        'urdf', 'srr.xacro'
    )

    robot_description_config = xacro.process_file(
        srr_xacro,
        mappings={
            'robot_type':       robot_type_str,
            'hand':             load_gripper_str,
            'gazebo_effort':    'false',
            'rail_joint_type':  'fixed',
        }
    )

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml()}],
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

    robot_type   = LaunchConfiguration('robot_type')
    load_gripper = LaunchConfiguration('load_gripper')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper]
    )

    # Gazebo — resource path : franka_description ET franka_sonde (meshes sonde)
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

    # Contrôleurs — même pattern que les exemples Franka officiels
    # (ExecuteProcess + ros2 control load_controller)
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_position_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_position_example_controller'],
        output='screen'
    )

    # joint_state_publisher : fournit rail_joint = 0.0
    # (rail_joint n'a pas d'interface ros2_control, joint_state_broadcaster
    #  ne le publie pas — joint_state_publisher comble ce manque)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['joint_states'], 'rate': 30}],
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        # spawn_robot fini → charger joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_jsb],
            )
        ),
        # jsb actif → charger le contrôleur de position
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_position_ctrl],
            )
        ),
    ])
