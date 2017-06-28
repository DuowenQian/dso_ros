cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  rosbag
)

# set(EXECUTABLE_OUTPUT_PATH ＄{PROJECT_SOURCE_DIR}/bin)

set(DSO_PATH $ENV{DSO_PATH})
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV 2.4 REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses Installed package
include_directories($(GTSAM_INCLUDE_DIR))
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
# find_library(DSO_LIBRARY dsoDebug ${DSO_PATH}/build/lib)
set(DSO_LIBRARY ${DSO_PATH}/build/lib/libdsoDebug.so)
message("DSO_LIB " ${DSO_LIBRARY})

find_package(Boost COMPONENTS system thread) 

# SSE flags
# rosbuild_check_for_sse()
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)

set(SOURCE_FILES         
  src/main.cpp
  src/MsgSync/MsgSynchronizer.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/MsgSync
  
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  $(GTSAM_INCLUDE_DIR)
  
)  

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES active_slam_teleop
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(dso_live src/main.cpp ${SOURCE_FILES})
add_dependencies(dso_live ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dso_live 
	${DSO_LIBRARY} 
	gtsam
	${Pangolin_LIBRARIES} 
	${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
	# boost_system boost_thread 
)

