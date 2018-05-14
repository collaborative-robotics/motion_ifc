cmake_minimum_required(VERSION 2.8.3)
project(motion_ifc)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (HDR_FILES include/${PROJECT_NAME}/Interpolate.h
               include/${PROJECT_NAME}/Controllers.h
               include/${PROJECT_NAME}/DataConversion.h)

set (SRC_FILES src/Interpolate.cpp
               src/Controllers.cpp
               src/DataConversion.cpp)
add_executable(${PROJECT_NAME}_node ${SRC_FILES} ${HDR_FILES})
add_executable(${PROJECT_NAME}_conversion_test src/ConversionTest.cpp)

 target_link_libraries(${PROJECT_NAME}_node
     ${catkin_LIBRARIES})
 target_link_libraries(${PROJECT_NAME}_conversion_test
    ${catkin_LIBRARIES})

