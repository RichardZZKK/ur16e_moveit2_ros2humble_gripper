cmake_minimum_required(VERSION 3.8)
project(ur16e_move_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs
)

add_executable(move_to_pose_server src/move_to_pose_server.cpp)
ament_target_dependencies(move_to_pose_server
  rclcpp rclcpp_action moveit_ros_planning_interface geometry_msgs sensor_msgs
)

add_executable(suction_gripper_mtp_server src/suction_gripper_mtp.cpp)
ament_target_dependencies(suction_gripper_mtp_server
  rclcpp rclcpp_action moveit_ros_planning_interface geometry_msgs sensor_msgs 
  std_srvs controller_manager_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_pose_server "${cpp_typesupport_target}")
target_link_libraries(suction_gripper_mtp_server "${cpp_typesupport_target}")

install(TARGETS move_to_pose_server suction_gripper_mtp_server DESTINATION lib/${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
    scripts/test_suction_gripper.py
    scripts/mock_perception_server.py
    scripts/send_goal.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other resources
install(DIRECTORY launch action config DESTINATION share/${PROJECT_NAME})

# Install documentation
install(FILES 
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
