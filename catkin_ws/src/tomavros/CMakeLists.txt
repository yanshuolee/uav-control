cmake_minimum_required(VERSION 3.0.2)
project(tomavros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/tomavros_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
