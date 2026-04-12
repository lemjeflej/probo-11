# gazebo_complet.launch.py
# =========================
# Séquence de lancement :
#   1. Gazebo vide
#   2. robot_state_publisher  (srr.xacro → /robot_description)
#   3. spawn robot + table_rail (xacro processé avec rail_position) + patient
#   4. joint_state_broadcaster activé
#   5. fr3_arm_controller activé  (JointTrajectoryController position)
#   6. cartesian_commander démarré (pose ready + écoute /target_pose)

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_table_rail_description(context: LaunchContext, rail_position):
    rail_position_str = context.perform_substitution(rail_position)

    table_rail_xacro = os.path.join(
        get_package_share_directory('franka_sonde'),
        'urdf', 'table_rail_visual.xacro'
    )
    xml = xacro.process_file(
        table_rail_xacro,
        mappings={'rail_position': rail_position_str}
    ).toxml()

    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', xml, '-name', 'table_rail'],
        output='screen',
        name='spawn_table_rail',
    )]


def get_robot_spawn_pose(context: LaunchContext, rail_position):
    """Calcule la pose de spawn du robot : y = -0.85 + rail_position."""
    rail_pos_str = context.perform_substitution(rail_position)
    y = -0.85 + float(rail_pos_str)
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'probo11',
            '-y',     f'{y:.4f}',
        ],
        output='screen',
        name='spawn_robot',
    )]


def get_curseur_spawn(context: LaunchContext, rail_position):
    """Calcule la pose de spawn du curseur rouge : y = -0.85 + rail_position, z = 1.03."""
    rail_pos_str = context.perform_substitution(rail_position)
    y = -0.85 + float(rail_pos_str)
    curseur_urdf = os.path.join(
        get_package_share_directory('franka_sonde'),
        'urdf', 'rail_curseur.urdf'
    )
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', curseur_urdf,
            '-name', 'rail_curseur',
            '-y',    f'{y:.4f}',
            '-z',    '1.03',
        ],
        output='screen',
        name='spawn_curseur',
    )]


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

    spawn_robot = OpaqueFunction(
        function=get_robot_spawn_pose,
        args=[rail_position]
    )

    spawn_curseur = OpaqueFunction(
        function=get_curseur_spawn,
        args=[rail_position]
    )

    spawn_table_rail = OpaqueFunction(
        function=get_table_rail_description,
        args=[rail_position]
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

    import os as _os
    carte_results = _os.path.join(
        _os.path.expanduser('~'),
        'robotics', 'probo-11', 'src', 'carte', 'results'
    )

    cartesian_commander = TimerAction(
        period=3.0,
        actions=[Node(
            package='controleurs',
            executable='cartesian_commander',
            name='cartesian_commander',
            output='screen',
            parameters=[{
                'move_duration': 5.0,
                'ik_max_iter':   200,
                'ik_tolerance':  1e-5,
            }],
        )]
    )

    rail_mover = TimerAction(
        period=3.0,
        actions=[Node(
            package='controleurs',
            executable='rail_mover',
            name='rail_mover',
            output='screen',
            parameters=[{'initial_rail_position': rail_position}],
        )]
    )

    mission_coordinator = TimerAction(
        period=4.0,
        actions=[Node(
            package='controleurs',
            executable='mission_coordinator',
            name='mission_coordinator',
            output='screen',
            parameters=[{'carte_results_dir': carte_results}],
        )]
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        rail_position_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_table_rail,
        spawn_curseur,
        spawn_patient,
        rail_mover,
        mission_coordinator,
        TimerAction(period=8.0,  actions=[load_jsb]),
        TimerAction(period=10.0, actions=[load_arm_ctrl]),
        TimerAction(period=13.0, actions=[cartesian_commander]),
    ])
