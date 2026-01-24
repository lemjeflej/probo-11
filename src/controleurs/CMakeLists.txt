
cmake_minimum_required(VERSION 3.8)
project(controleurs)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controleur_rail src/controleur_rail.cpp)
ament_target_dependencies(controleur_rail rclcpp sensor_msgs)


install(TARGETS controleur_rail
  DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY .
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
