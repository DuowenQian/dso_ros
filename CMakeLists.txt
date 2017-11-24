cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

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
  tf
  message_filters
)

# set(EXECUTABLE_OUTPUT_PATH ＄{PROJECT_SOURCE_DIR}/bin)

set(DSO_PATH /home/duo/dso)
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV 2.4 REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_library(DSO_LIBRARY dsoDebug ${DSO_PATH}/build/lib)
message("DSO_LIB " ${DSO_LIBRARY})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES active_slam_teleop
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs cv_bridge tf message_filters
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)

set(CMAKE_CXX_FLAGS "-std=c++14 -march=native")

set(ros_IOWrapper_source
  ${PROJECT_SOURCE_DIR}/src/ROSOutputPublisher.cpp
)

set(SOURCE_FILES         
  src/main.cpp
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)  


add_executable(dso_live src/main.cpp ${SOURCE_FILES})
add_dependencies(dso_live ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(ros_IOWrapper ${ros_IOWrapper_source})
target_link_libraries(dso_live ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} ros_IOWrapper boost_system boost_thread tf)

