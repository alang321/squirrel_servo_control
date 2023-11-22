cmake_minimum_required(VERSION 2.8.3)
project(squirrel_servo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs  # Include any other message dependencies
  message_generation
)

catkin_python_setup()


add_message_files(DIRECTORY msg FILES motor_speed.msg servo_feedback.msg servo_position.msg servo_speed.msg)


generate_messages(DEPENDENCIES std_msgs)

catkin_install_python(PROGRAMS
  src/squirrel_servo_node.py
  src/servo_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/squirrel_servo_control
)
install(FILES
  launch/test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/squirrel_servo_control
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


