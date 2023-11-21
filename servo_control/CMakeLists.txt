cmake_minimum_required(VERSION 3.0.2)
project(servo_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs roslaunch
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(servo_control_node src/servo_control_node.cpp)
target_link_libraries(servo_control_node ${catkin_LIBRARIES})


add_dependencies(servo_control_node ${catkin_EXPORTED_TARGETS})
