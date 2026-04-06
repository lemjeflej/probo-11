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
# initial_pose_controller  (RViz / debug)
# ----------------------------------------
add_executable(initial_pose_controller src/initial_pose_controller.cpp)
ament_target_dependencies(initial_pose_controller
    rclcpp
    sensor_msgs
    kdl_parser
    orocos_kdl
)
target_link_libraries(initial_pose_controller orocos-kdl)

# ----------------------------------------
# controleur_rail
# ----------------------------------------
add_executable(controleur_rail src/controleur_rail.cpp)
ament_target_dependencies(controleur_rail rclcpp sensor_msgs)

# ----------------------------------------
# test_node
# ----------------------------------------
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp sensor_msgs)

# ----------------------------------------
# Install
# ----------------------------------------
install(TARGETS
    cartesian_commander
    initial_pose_controller
    controleur_rail
    test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/pose_gui.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
