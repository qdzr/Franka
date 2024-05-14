cmake_minimum_required(VERSION 3.4)
project(franka_pick_place)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  control_msgs
  actionlib
  sensor_msgs
  xmlrpcpp
  # Eigen3
  actionlib_msgs
)
include_directories("/usr/include/eigen3")
find_package(Franka 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)
add_action_files(
  DIRECTORY action
  FILES Grasp.action
        Homing.action
        Stop.action
        Move.action
)


add_message_files(
  DIRECTORY msg
  FILES GraspEpsilon.msg
)

generate_messages(DEPENDENCIES actionlib_msgs)
include_directories("/usr/include/eigen3")

#target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_pick_place
  CATKIN_DEPENDS roscpp
                 message_runtime
                 control_msgs
                 actionlib
                 libfranka
                 sensor_msgs
                 xmlrpcpp
                 actionlib_msgs
  DEPENDS Franka
)

#add_library(franka_gripper
#  src/franka_gripper.cpp
#)
#add_library(examples_common STATIC src/examples_common.cpp)
add_executable(franka_pick_place
  src/franka_pick_place.cpp
)
add_executable(move_to_start_pose
  src/move_to_start_pose.cpp
)
add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

#target_link_libraries(
 # ${catkin_LIBRARIES}
#)

target_include_directories(franka_pick_place SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(move_to_start_pose SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_pick_place PUBLIC
  include
)
target_include_directories(move_to_start_pose PUBLIC
  include
)
add_dependencies(franka_pick_place move_to_start_pose
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
 # franka_gripper
)

target_link_libraries(franka_pick_place ${Eigen3_LIBRARY} ${Franka_LIBRARIES} ${catkin_LIBRARIES}
#   franka_gripper
)
target_link_libraries(move_to_start_pose ${Eigen3_LIBRARY} ${Franka_LIBRARIES} ${catkin_LIBRARIES}
#   franka_gripper
)
target_include_directories(franka_pick_place SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(move_to_start_pose SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
## Installation
install(TARGETS franka_pick_place move_to_start_pose
               # franka_pick_place
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_pick_place move_to_start_pose FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_pick_place move_to_start_pose
    FILES ${SOURCES}
   # DEPENDS franka_pick_place
  )
endif()
