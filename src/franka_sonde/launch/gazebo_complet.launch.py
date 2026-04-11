# gazebo_complet.launch.py
# =========================
# Séquence de lancement :
#   1. Gazebo vide
#   2. robot_state_publisher  (srr.xacro → /robot_description)
#   3. joint_state_publisher  (fournit rail_joint=0)
#   4. spawn du robot dans Gazebo
#   5. joint_state_broadcaster activé
#   6. fr3_arm_controller activé  (JointTrajectoryController position)
#   7. cartesian_commander démarré
#      → envoie la trajectoire vers la pose "ready" au démarrage
#      → attend ensuite des poses sur /target_pose

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
            'robot_type':    robot_type_str,
            'hand':          load_gripper_str,
            'gazebo_effort': 'false',
        }
    )

    # The Franka xacro hardcodes franka_gazebo_bringup's controllers yaml into
    # the Gazebo plugin <parameters> element.  In Humble, a controller must be
    # declared in that yaml at CM startup — ros2 param set after the fact is
    # not reliably forwarded to dynamically loaded controller nodes.
    # We swap the path to our augmented yaml (same content + fr3_arm_controller).
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

    robot_type   = LaunchConfiguration('robot_type')
    load_gripper = LaunchConfiguration('load_gripper')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper]
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

    # rail_joint n'a pas d'interface ros2_control (joint fixe) :
    # joint_state_publisher le publie à 0.0 pour que robot_state_publisher
    # puisse calculer /tf correctement.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['joint_states'], 'rate': 30}],
    )

    # joint_state_broadcaster : type connu de franka_gazebo_controllers.yaml
    # → on peut utiliser load_controller directement
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # fr3_arm_controller :
    # Le type et les params sont définis dans franka_gazebo_controllers.yaml
    # (version augmentée installée par franka_sonde qui shadow l'originale).
    # Le CM Gazebo charge ce yaml au démarrage → load_controller suffit.
    load_arm_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'fr3_arm_controller'],
        output='screen'
    )

    load_rail_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'rail_controller'],
        output='screen'
    )

    # cartesian_commander :
    # - se connecte à /joint_states et /fr3_arm_controller/follow_joint_trajectory
    # - à l'init, envoie automatiquement la trajectoire vers la pose "ready"
    # - attend ensuite des commandes sur /target_pose
    # Délai de 3s pour laisser fr3_arm_controller s'activer complètement.
    cartesian_commander = TimerAction(
        period=3.0,
        actions=[Node(
            package='controleurs',
            executable='cartesian_commander',
            name='cartesian_commander',
            output='screen',
            parameters=[{
                'move_duration': 5.0,   # durée du mouvement vers ready (s)
                'ik_max_iter':   200,
                'ik_tolerance':  1e-5,
            }],
        )]
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
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
                on_exit=[load_rail_ctrl],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rail_ctrl,
                on_exit=[cartesian_commander],
            )
        ),
    ])
