cmake_minimum_required(VERSION 3.0.2)
project(panda_demo)
 
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  gpd_ros
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.9.1  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
catkin_package( 
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  pcl_ros
  moveit_core
  message_filters 
  pcl_conversions
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  gpd_ros  # 添加gpd_ros依赖
)

 
catkin_package()
 
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${gpd_ros_INCLUDE_DIRS}
)
  
 
 

 
# GPD_method_grasp ##
add_executable(gpd_method ${PROJECT_SOURCE_DIR}/grasp/gpd_method/src/main.cpp)
add_dependencies(gpd_method ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpd_method ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
# GPD_method_grasp ##
add_executable(cloud_fusion_node ${PROJECT_SOURCE_DIR}/grasp/gpd_method/src/cloud_fusion_node.cpp)
add_dependencies(cloud_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_fusion_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
  

install(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 