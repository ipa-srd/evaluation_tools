cmake_minimum_required(VERSION 2.8.3)
project(ipa_map_comparison)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    message_generation
    nav_msgs
    roscpp
    roslib
    rospy
    rostest
    std_msgs)

add_service_files(DIRECTORY srv FILES
                  StartMapEval.srv)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ipa_map_comparison
  message_runtime
)

include_directories(include)

 add_library(ipa_map_comparison
   src/ipa_map_comparison_node.cpp
 )

 add_dependencies(ipa_map_comparison ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(ipa_map_comparison_node src/ipa_map_comparison_node.cpp)

 target_link_libraries(ipa_map_comparison_node
   ${catkin_LIBRARIES}
 )
