cmake_minimum_required(VERSION 3.10.2)
project(capnzero_eval)

add_definitions(-std=c++11)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(Threads)
find_package(catkin REQUIRED COMPONENTS capnzero fsystem roscpp message_generation)

############# Evaluation Message ROS
add_message_files(FILES EvalMessageRos.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES capnzero-eval-msgs
  CATKIN_DEPENDS capnzero fsystem roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############# Evaluation Software / Msg Sender
add_executable(Evaluate
  src/CapnZeroEvaluation.cpp
  src/ExperimentLog.cpp
)
target_link_libraries(Evaluate
  capnzero-eval-msgs
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
add_dependencies(Evaluate capnzero_eval_generate_messages_cpp)

############# Evaluation Software / Msg Echo
add_executable(Parrot
  src/CapnZeroParrot.cpp
)
target_link_libraries(Parrot
  capnzero-eval-msgs
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
add_dependencies(Parrot capnzero_eval_generate_messages_cpp)

############# Evaluation Message CapnZero
find_package(CapnProto REQUIRED)
set(capnzero_eval_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE capnzero_eval_msgs ${capnzero_eval_message_folder} *.capnp)
set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero-eval-msgs")
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero-eval-msgs")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnzero_eval_msgs})

add_library(capnzero-eval-msgs
  ${CAPNP_SRCS}
)
