cmake_minimum_required(VERSION 2.8.3)
project(multi_uav)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED
  
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime catkin
)
include_directories(
  include ${catkin_INCLUDE_DIRS}${catkin_INCLUDE_DIRS})

add_executable(initvel src/initvel.cpp)
add_executable(rotate src/rotate.cpp)
add_executable(override src/override.cpp)
add_executable(front_vel src/front_vel.cpp)
add_executable(control_yaw src/control_yaw.cpp)
add_executable(ground_station src/ground_station.cpp)
add_executable(flock src/flock.cpp)
add_executable(linearFlock src/linearFlock.cpp)
add_executable(levy src/levy.cpp)
add_executable(formation_creating src/formation_creating.cpp) ##----ADDITIONAL
add_executable(test_node_pub src/test_node_pub.cpp) ###----additional
add_executable(leader_follower src/leader_follower.cpp) ###----additional
add_executable(pid_control src/pid_controller.cpp) ###----additional
add_executable(artificial_potential_field src/artificial_potential_field.cpp) ###-----additional
add_executable(APF_5uav src/artificial_potential_field_5uav.cpp)  ##----additional


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(random_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(random_vel_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(initvel ${catkin_LIBRARIES})
target_link_libraries(rotate ${catkin_LIBRARIES})
target_link_libraries(override ${catkin_LIBRARIES})
target_link_libraries(front_vel ${catkin_LIBRARIES})
target_link_libraries(control_yaw ${catkin_LIBRARIES})
target_link_libraries(ground_station ${catkin_LIBRARIES})
target_link_libraries(flock ${catkin_LIBRARIES})
target_link_libraries(linearFlock ${catkin_LIBRARIES})
target_link_libraries(levy ${catkin_LIBRARIES})
target_link_libraries(test_node_pub ${catkin_LIBRARIES}) ## ---additional
target_link_libraries(formation_creating ${catkin_LIBRARIES}) ##----ADDITIONAL
target_link_libraries(leader_follower ${catkin_LIBRARIES}) ##---additional
target_link_libraries(pid_control ${catkin_LIBRARIES}) ##----additional
target_link_libraries(artificial_potential_field ${catkin_LIBRARIES}) ##----ADDITIONAL
target_link_libraries(APF_5uav ${catkin_LIBRARIES}) ##----ADDITIONAL

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)
# add_message_files(
#   FILES
# )

#generate_messages(DEPENDENCIES
#  std_msgs
#)
