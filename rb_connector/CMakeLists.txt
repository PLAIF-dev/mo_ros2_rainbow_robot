cmake_minimum_required(VERSION 3.5)
project(rb_connector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RbCommand.msg"
  "msg/RbData.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(${PROJECT_NAME}_lib SHARED
  src/rb_connector.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rb_connector>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}"
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_msgs
  actionlib_msgs
  moveit_msgs
  control_msgs
  moveit_ros_planning_interface
)

add_executable(${PROJECT_NAME}_node src/rb_connector_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_node
  rb_connector_lib
)

# rosidl_target_interfaces(rb_connector_lib
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# rosidl_target_interfaces(${PROJECT_NAME}_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
#   EXPORT export_${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include
# )

# install(DIRECTORY
#   launch
#   msg
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(rb_connector_lib)
ament_package()
