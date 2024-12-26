cmake_minimum_required(VERSION 3.0.2)
project(camera_ros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  camera_info_manager
  cv_bridge
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera>=0.1)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_ros utils
  CATKIN_DEPENDS roscpp sensor_msgs camera_info_manager cv_bridge
  DEPENDS libcamera
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${libcamera_INCLUDE_DIRS}
)

# Library with common utility functions for type conversions
add_library(utils
  src/clamp.cpp
  src/cv_to_pv.cpp
  src/format_mapping.cpp
  src/parameter_conflict_check.cpp
  src/pretty_print.cpp
  src/pv_to_cv.cpp
  src/types.cpp
  src/type_extent.cpp
)
target_link_libraries(utils ${libcamera_LIBRARIES} ${catkin_LIBRARIES})

# Camera ROS1 node
add_executable(camera_node src/CameraNode.cpp)
target_link_libraries(camera_node
  utils
  ${libcamera_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
  utils
  camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_python()
endif()
