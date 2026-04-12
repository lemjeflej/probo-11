cmake_minimum_required(VERSION 3.8)
project(controleurs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

# ----------------------------------------
# cartesian_commander  (nœud IK + action)
# ----------------------------------------
add_executable(cartesian_commander src/cartesian_commander.cpp)
ament_target_dependencies(cartesian_commander
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    std_msgs
    trajectory_msgs
    control_msgs
    kdl_parser
    orocos_kdl
)
target_link_libraries(cartesian_commander orocos-kdl)

# ----------------------------------------
# Install
# ----------------------------------------
install(TARGETS
    cartesian_commander
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/pose_gui.py
    DESTINATION lib/${PROJECT_NAME}
)

# Scripts sans extension .py pour ros2 run / Node(executable=...)
install(PROGRAMS scripts/rail_mover.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME rail_mover
)
install(PROGRAMS scripts/mission_coordinator.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME mission_coordinator
)

ament_package()
