cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(asr_follow_person_bt_node SHARED src/visual_behavior/FollowPerson.cpp)
add_library(asr_detect_person_bt_node SHARED src/visual_behavior/DetectPerson.cpp)
add_library(asr_turn_bt_node SHARED src/visual_behavior/Turn.cpp)
add_library(asr_percieve_person_bt_node SHARED src/visual_behavior/PercievePerson.cpp)
list(APPEND plugin_libs
  asr_follow_person_bt_node
  asr_detect_person_bt_node
  asr_turn_bt_node
  asr_percieve_person_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(follow_person_node 
  src/follow_person_node.cpp
)
target_link_libraries(follow_person_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})


roslint_cpp(
  src/follow_person_node.cpp
  include/visual_behavior/FollowPerson.h src/visual_behavior/FollowPerson.cpp
  include/visual_behavior/DetectPerson.h src/visual_behavior/DetectPerson.cpp
  include/visual_behavior/PercievePerson.h src/visual_behavior/PercievePerson.cpp
  include/visual_behavior/Turn.h src/visual_behavior/Turn.cpp
)
