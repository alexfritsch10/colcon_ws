cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME zed-package)
project(${PROJECT_NAME})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "/root/zed-open-capture/cmake")

# Log CMAKE_SOURCE_DIR
message("CMAKE_SOURCE_DIR is: ${CMAKE_SOURCE_DIR}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(HIDAPI REQUIRED)

# Include directories for zed-open-capture
include_directories(
  /root/zed-open-capture/include
  ${HIDAPI_INCLUDE_DIRS} 
)

link_directories(/root/zed-open-capture/build)

# Stereo Node ---------------------->
add_executable(${PROJECT_NAME}_stereo_node src/zed_stereo_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_stereo_node rclcpp sensor_msgs image_transport cv_bridge OpenCV)
# Link the executable with the necessary libraries
target_link_libraries(${PROJECT_NAME}_stereo_node
  zed_open_capture  # Link against the zed_open_capture library
  ${OpenCV_LIBS}    # Link against OpenCV libraries, if applicable
)

add_definitions(-DSENSORS_MOD_AVAILABLE)
add_definitions(-DVIDEO_MOD_AVAILABLE)

install(TARGETS ${PROJECT_NAME}_stereo_node
  DESTINATION lib/${PROJECT_NAME})

# RGB Node ---------------------->
  add_executable(${PROJECT_NAME}_rgbd_node src/zed_rgbd_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_rgbd_node rclcpp sensor_msgs image_transport cv_bridge OpenCV)
# Link the executable with the necessary libraries
target_link_libraries(${PROJECT_NAME}_rgbd_node
  zed_open_capture  # Link against the zed_open_capture library
  ${OpenCV_LIBS}    # Link against OpenCV libraries, if applicable
)

add_definitions(-DSENSORS_MOD_AVAILABLE)
add_definitions(-DVIDEO_MOD_AVAILABLE)

install(TARGETS ${PROJECT_NAME}_rgbd_node
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
