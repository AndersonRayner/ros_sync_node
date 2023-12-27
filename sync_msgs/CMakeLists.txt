cmake_minimum_required(VERSION 3.0.2)
project(sync_msgs)

set(DEPEND_MSGS )

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
   sensorHealth.msg
   sensorHealthArray.msg
   sensorHealthArrayNamed.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
