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

set (HDR_FILES include/${PROJECT_NAME}/Trajectory.h
               include/${PROJECT_NAME}/Controllers.h
               include/${PROJECT_NAME}/DataConversion.h
               include/${PROJECT_NAME}/Communication.h
               include/${PROJECT_NAME}/MotionCmdIfc.h
               include/${PROJECT_NAME}/RobotCmdIfc.h
               include/${PROJECT_NAME}/RobotStateIfc.h
               include/${PROJECT_NAME}/crtkCommon.h
               include/${PROJECT_NAME}/FcnHandle.h)

set (SRC_FILES src/Controllers.cpp
               src/DataConversion.cpp
               src/TrajectoryTest.cpp)

add_executable(${PROJECT_NAME}_node ${SRC_FILES} ${HDR_FILES})
add_executable(conversion_test src/ConversionTest.cpp)
add_executable(fcn_handle_test src/FcnHandleTest.cpp)
add_executable(communication_test src/CommunicationTest.cpp include/${PROJECT_NAME}/Communication.h)

 target_link_libraries(${PROJECT_NAME}_node
     ${catkin_LIBRARIES})
 target_link_libraries(conversion_test
    ${catkin_LIBRARIES})
 target_link_libraries(fcn_handle_test
    ${catkin_LIBRARIES})
 target_link_libraries(communication_test
    ${catkin_LIBRARIES})
