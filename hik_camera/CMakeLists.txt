cmake_minimum_required(VERSION 3.0.2)
project(hik_camera)

# set(CMAKE_BUILD_TYPE "Release")
# #set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")


find_package(catkin REQUIRED COMPONENTS
    roslaunch
    tf
    roscpp
    cv_bridge
    image_transport
    # msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    visualization_msgs
    camera_info_manager
)
find_package(OpenCV REQUIRED QUIET)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# # Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    message_runtime
)


###########
## Build ##
###########

# include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES
    src/HikCamera.cpp
)

file(GLOB_RECURSE HK_LIB
    lib/*.so
)

add_executable(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${HK_LIB}
)

#############
## Install ##
#############

# message(status " LIBRARY = ${LIBRARY}")
# message(status "CATKIN_PACKAGE_LIB_DESTINATION=${CATKIN_PACKAGE_LIB_DESTINATION}")
# message(status "CATKIN_PACKAGE_LIB_DESTINATION=${CATKIN_PACKAGE_BIN_DESTINATION}")

# 指定安装目标和目的地

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "*"
)

# 安装launch文件和配置文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)