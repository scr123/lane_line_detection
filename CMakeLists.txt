cmake_minimum_required(VERSION 2.8.3)
project(lane_line_detection)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
  std_msgs
	sensor_msgs
  geometry_msgs
  visualization_msgs
  rviz_visual_tools
	cv_bridge
	image_transport
  robot_state_publisher
  dynamic_reconfigure
)

add_message_files(
   FILES
   LaneLine.msg
   LaneLines.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

 generate_dynamic_reconfigure_options(
   cfg/FilterParams.cfg
 )

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
)

###########
## Build ##
###########

### Lane Line Detection Library

add_library(lld_api
  src/lane_line_detection.cpp
  src/least_squares.cpp
)

target_include_directories(lld_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(lld_api
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(lld_api
  ${catkin_EXPORTED_TARGETS}
)

### ROS Interface

include_directories(
 include
 ${EIGEN3_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/lane_line_detection.cpp
  src/least_squares.cpp
 	src/ros_interface_node.cpp
 	src/ros_interface.cpp
)

target_include_directories(${PROJECT_NAME}_node
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
	${catkin_EXPORTED_TARGETS}
)

 target_link_libraries(${PROJECT_NAME}_node
   ${EIGEN3_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

#############
## Testing ##
#############

catkin_add_gtest(lane_line_detection_test test/lane_line_detection_test.cpp
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/test/images/")

target_include_directories(lane_line_detection_test
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(lane_line_detection_test lld_api)

add_dependencies(lane_line_detection_test
  ${catkin_EXPORTED_TARGETS}
)
