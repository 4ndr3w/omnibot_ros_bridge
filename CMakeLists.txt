cmake_minimum_required(VERSION 2.8.3)
project(omnibot_bridge)
find_package(catkin REQUIRED roscpp std_msgs tf)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

add_executable(frc_comm src/frc_comm.cpp)
target_link_libraries(frc_comm ${catkin_LIBRARIES} z)


