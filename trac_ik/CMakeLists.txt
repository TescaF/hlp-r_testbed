cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_wrapper)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs trac_ik_lib)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")

add_service_files(
  FILES
  IKHandler.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs trac_ik_lib
  DEPENDS Boost orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${Eigen3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(trac_ik_wrapper src/ik_handler.cpp)
target_link_libraries(trac_ik_wrapper ${catkin_LIBRARIES})
add_dependencies(trac_ik_wrapper trac_ik_wrapper_generate_messages_cpp)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS trac_ik_wrapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

