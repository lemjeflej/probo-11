cmake_minimum_required(VERSION 3.8)
project(controleurs)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

#----------------------------------------
# Controleur rail
#----------------------------------------
add_executable(controleur_rail src/controleur_rail.cpp)
ament_target_dependencies(controleur_rail rclcpp sensor_msgs)

#----------------------------------------
# Initial Pose Controller
#----------------------------------------
add_executable(initial_pose_controller src/initial_pose_controller.cpp)
ament_target_dependencies(initial_pose_controller 
    rclcpp 
    sensor_msgs 
    kdl_parser
    orocos_kdl
)
target_link_libraries(initial_pose_controller orocos-kdl)

#----------------------------------------
# Test Node
#----------------------------------------
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node 
    rclcpp 
    sensor_msgs 
)
#----------------------------------------
# Install targets
#----------------------------------------
install(TARGETS initial_pose_controller test_node controleur_rail
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
