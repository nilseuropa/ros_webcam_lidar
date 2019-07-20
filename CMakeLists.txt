cmake_minimum_required(VERSION 2.8.3)
project(webcam_lidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  urdf
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  dynamic_reconfigure
  genmsg
  message_generation
  visualization_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)

generate_dynamic_reconfigure_options(
  #cfg/webcam_lidar.cfg
  cfg/color_filter.cfg
)

catkin_package(
   CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
##  Build  ##
#############

add_executable(soft_color_filter src/soft_color_filter.cpp)
add_dependencies(soft_color_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(soft_color_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(triangulation_node src/triangulation_node.cpp)
#target_link_libraries(triangulation_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)


#############
## Install ##
#############

install(TARGETS soft_color_filter
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
