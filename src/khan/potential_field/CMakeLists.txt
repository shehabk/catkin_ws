cmake_minimum_required(VERSION 2.8.3)
project(potential_field)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs 
  message_filters
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs
  sensor_msgs
) 
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(potential_field  src/potential_field.cpp)
target_link_libraries(potential_field ${catkin_LIBRARIES})
target_link_libraries(potential_field ${OpenCV_LIBS} ${BULLET_LIBRARIES})
add_executable(potential_field_wall  src/potential_field_wall.cpp)
target_link_libraries(potential_field_wall ${catkin_LIBRARIES})
target_link_libraries(potential_field_wall ${OpenCV_LIBS} ${BULLET_LIBRARIES})
add_dependencies(grid_mapper ${PROJECT_NAME}_generate_messages_cpp)
