cmake_minimum_required(VERSION 2.8.3)
project(limo_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_dev
  gazebo_msgs
  actionlib
  sensor_msgs
  gazebo_dev
  message_generation
  nodelet
  rosconsole
)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_action_files(
  FILES
  StopResumeExploration.action
  LaunchRobot.action
  DistanceQuery.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  LIBRARIES  gazebo_ros_ackerman 
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    actionlib 
    gazebo_ros 
    message_runtime 
    gazebo_msgs 
    nodelet  
    rosconsole 
    std_msgs
)


add_executable(distance_calculator src/distance_calculator.cpp)
add_dependencies(distance_calculator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_calculator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(battery_simulator src/battery_simulator.cpp)
add_dependencies(battery_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_simulator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(simulation_manager_server src/simulation_manager_server.cpp)
add_dependencies(simulation_manager_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_manager_server ${catkin_LIBRARIES})




install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


install(DIRECTORY xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

