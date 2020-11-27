cmake_minimum_required(VERSION 2.8.3)
project(wrs_run)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs 
    gazebo_ros
    gazebo_ros_link_attacher
    moveit_jog_arm
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers  
  DEPENDS
    EIGEN3
    gazebo_ros
    gazebo_ros_link_attacher
    geometry_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

message(WARNING "{GAZEBO_LIBRARIES}: ${GAZEBO_LIBRARIES}")
message(WARNING "{GAZEBO_LIBRARY_DIRS}: ${GAZEBO_LIBRARY_DIRS}")



###########
## Build ##
###########
include_directories(include)
link_directories(${catkin_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/functions.cpp)


add_executable(pick_place_simu src/pick_place_simu.cpp)
add_dependencies(pick_place_simu ${catkin_EXPORTED_TARGETS})

add_executable(pick_place_real src/pick_place_real.cpp)
add_dependencies(pick_place_real ${catkin_EXPORTED_TARGETS})

add_executable(ft_mean_filter src/ft_mean_filter.cpp)
add_executable(ft_difference src/ft_difference.cpp)
add_executable(ft_world_transform_simu src/ft_world_transform_simu.cpp)
add_executable(ft_world_transform_real src/ft_world_transform_real.cpp)

target_link_libraries(pick_place_simu ${PROJECT_NAME} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pick_place_real ${PROJECT_NAME} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(ft_mean_filter ${catkin_LIBRARIES})
target_link_libraries(ft_difference ${catkin_LIBRARIES})
target_link_libraries(ft_world_transform_simu ${catkin_LIBRARIES})
target_link_libraries(ft_world_transform_real ${catkin_LIBRARIES})


