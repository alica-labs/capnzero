cmake_minimum_required(VERSION 2.8.3)
project(discovery_msgs)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

# Define where to find modules for ZeroMQ and UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(CapnProto REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES discovery_msgs
)

# make src an include directory, because everything is generated into the src folder
include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
 
## Autogenerate the c++ capnproto files
set(capnp_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE capnp_messages ${capnp_message_folder} *.capnp)

set(CAPNPC_SRC_PREFIX msg)
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

#set(position "")
#set(length "")
#string(LENGTH ${CAPNP_HDRS} length)
#string(FIND ${CAPNP_HDRS} "/" position REVERSE)
#set(sub "")
#string(SUBSTRING ${CAPNP_HDRS} ${position}+1 ${length}-1  sub)
#add_custom_target(MoveHeaderToIncludeFolder ALL
#  COMMAND ${CMAKE_COMMAND} -E copy ${CAPNP_HDRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${sub}
#  COMMAND ${CMAKE_COMMAND} -E remove ${CAPNP_HDRS}
#  DEPENDS ${CAPNP_HDRS}
#)

## Declare the C++ capnproto message library
add_library(${PROJECT_NAME}
  ${CAPNP_SRCS}
)

#add_dependencies(${PROJECT_NAME} MoveHeaderToIncludeFolder)
