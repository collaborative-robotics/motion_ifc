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
               include/${PROJECT_NAME}/MotionCmd.h
               include/${PROJECT_NAME}/RobotCmd.h
               include/${PROJECT_NAME}/RobotState.h
               include/${PROJECT_NAME}/crtkCommon.h
               include/${PROJECT_NAME}/FcnHandle.h
               include/${PROJECT_NAME}/WatchDog.h)

set (SRC_FILES src/crtkCommon.cpp
               src/Controllers.cpp
               src/ControllerData.cpp
               src/Communication.cpp
               src/DataConversion.cpp
               src/Trajectory.cpp
               src/MotionCmd.cpp
               src/RobotState.cpp
               src/RobotCmd.cpp)

add_library(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES})

foreach(TESTS Conversion FcnHandle Communication Controller Trajectory)
    add_executable(${TESTS}_test tests/${TESTS}Test.cpp)
    target_link_libraries(${TESTS}_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach()

