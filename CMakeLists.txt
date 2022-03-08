cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
  darknet_ros_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/visual_behavior/follow_ball.cpp
)

add_executable(follow_ball_node src/follow_ball_node.cpp)
target_link_libraries(follow_ball_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/follow_ball_node.cpp
 src/visual_behavior/follow_ball.cpp
 include/follow_ball.h
)