cmake_minimum_required(VERSION 3.0.2)
project(camera_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
