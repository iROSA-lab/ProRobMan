cmake_minimum_required(VERSION 2.8.3)
project(ProRobMan)
find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
)
catkin_package()

add_executable(spawn_cubes scripts/spawn_cubes.py)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})