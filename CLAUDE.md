# PROBO-11: FR3 Robot with Linear Rail and Custom Sonde

**Purpose**: ROS 2 Humble simulation of Franka FR3 with 1.9m linear rail system and custom probe end-effector in Gazebo.

## Package Structure

- **franka_sonde** — Custom URDF, launch files, meshes, and RViz configs for rail+robot assembly
- **controleurs** — C++ custom controllers for rail and IK (KDL-based)
- **fr3_sonde_moveit_config** — MoveIt2 configuration, controllers.yaml, kinematics, planning
- **franka_description** — Official Franka URDF library (FR3, FP3, FER models) — **DO NOT MODIFY**
- **franka_ros2** — Core ROS 2 integration, Gazebo bridge, hardware interface — **DO NOT MODIFY**
- **libfranka** — Low-level C++ control library — **DO NOT MODIFY**

## Tech Stack

- **ROS 2 Humble** with Python launch files
- **Gazebo** (ros_gz_sim) for simulation
- **ros2_control** framework for controller management
- **MoveIt 2** for trajectory planning and motion control
- **KDL** (Kinematics and Dynamics Library) for IK solving
- **C++17** for custom controllers

## URDF & Launch

- **URDF Entry Point**: `franka_sonde/urdf/sonde_robot_rail.xacro`
- **Key Launch File**: `franka_sonde/launch/gazebo_test.launch.py`
- **Launch Parameters**:
  - `robot_type:=fr3` (options: fr3, fp3, fer)
  - `load_gripper:=false` (always false — custom sonde replaces gripper)
  - `namespace:=''` (optional namespace isolation)

## Controllers

- **fr3_arm_controller** — 7-DOF arm trajectory control (FollowJointTrajectory)
- **rail_controller** — 1-DOF linear rail control (FollowJointTrajectory)
- **joint_state_broadcaster** — Publishes joint states from hardware
- **joint_impedance_example_controller** — Example impedance control (loaded in Gazebo)

## Build Command

```bash
colcon build --packages-select franka_sonde controleurs fr3_sonde_moveit_config
```

## Critical Rules

- **Never modify** `franka_description`, `franka_ros2`, or `libfranka` (upstream packages)
- **arm_id prefix** is always `fr3` in controller names
- **load_gripper** launch argument is always `false` (custom sonde end-effector used instead)
- **Planning group** is `fr3_sur_rail` (8 joints: 1 rail + 7 arm)
- **SRDF file**: `franka_sonde/urdf/robot_complet.srdf.xacro`

## Custom Components

1. **Linear Rail** — Prismatic joint (0–1.9m travel), mounted on steel table
2. **Robot Carriage** — 0.2m × 0.2m platform that slides on rail
3. **Custom Sonde** — Probe tool (0.3kg) with DAE/STL meshes, tool TCP at 0.2m below base
