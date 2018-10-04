cmake_minimum_required(VERSION 2.8.3)
project(vicon_hand)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix -DCATKIN_BUILD")
add_definitions(-std=c++98)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
  mg_msgs
)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  handMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime mg_msgs vicon
    DEPENDS Eigen
    DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(hand_interp_node src/handMain.cpp src/handInterp.cpp)
add_executable(vicon_interp_node src/interpMain.cpp src/viconInterp.cpp)
add_dependencies(vicon_interp_node vicon_hand_generate_messages_cpp)
add_dependencies(vicon_interp_node vicon_generate_messages_cpp)
add_dependencies(vicon_interp_node mg_msgs_node_generate_messages_cpp)
add_dependencies(hand_interp_node vicon_hand_generate_messages_cpp)
add_dependencies(hand_interp_node vicon_generate_messages_cpp)
add_dependencies(hand_interp_node mg_msgs_generate_messages_cpp)
target_link_libraries(vicon_interp_node ${catkin_LIBRARIES})
target_link_libraries(hand_interp_node ${catkin_LIBRARIES})
