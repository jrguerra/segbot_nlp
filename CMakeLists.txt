## This is a test
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/command_module/command_node.cpp)

#rosbuild_add_library(LinguisticTree src/linguistic_tree/LinguisticTree.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(command_node src/command_module/command_node.cpp)
rosbuild_add_executable(interpreter_node src/interpreter/interpreter_node.cpp /nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/linguistic_tree/pugixml.cpp /nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/linguistic_tree/LinguisticElement.cpp /nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/linguistic_tree/LinguisticStructures.cpp /nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/linguistic_tree/LinguisticTree.cpp /nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/interpreter/Interpreter.cpp)
rosbuild_add_executable(debug_command_node_tester src/debug/debug_command_node_tester.cpp)
rosbuild_add_executable(debug_linguistic_tree src/debug/debug_linguistic_tree.cpp)
#rosbuild_add_executable(debug_linguistic_tree_node src/debug/debug_linguistic_tree.cpp)
#target_link_libraries(example ${PROJECT_NAME})
