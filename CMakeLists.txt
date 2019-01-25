cmake_minimum_required(VERSION 2.8.3)
project(hand_endpoint)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix -DCATKIN_BUILD")
set(CMAKE_CXX_STANDARD 11)

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
  oneHand.msg
  hands.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime mg_msgs nav_msgs
    DEPENDS Eigen
    DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(hand_endpoint_node src/main.cpp src/commander.cpp src/commanderActions.cpp src/poseContainer.cpp src/quadContainer.cpp src/handEndpoint.cpp)
add_dependencies(hand_endpoint_node mg_msgs_node_generate_messages_cpp)
add_dependencies(hand_endpoint_node hand_endpoint_generate_messages_cpp)
target_link_libraries(hand_endpoint_node ${catkin_LIBRARIES})
