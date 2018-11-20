cmake_minimum_required(VERSION 2.8.3)
project(capnzero)

## Use c++ 11x std
add_definitions(-std=c++11)

# Define where to find modules for ZeroMQ and UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

# this enables the draft part of the api
add_definitions(-DZMQ_BUILD_DRAFT_API=1)
find_package(ZeroMQ)

# Introduce CapnProto
find_package(CapnProto REQUIRED)

message("CapnProto Incl: ${CAPNP_INCLUDE_DIRS}")
message("CapnProto Lib: ${CAPNP_LIBRARIES}")

find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES capnzero
    DEPENDS ZeroMQ
  )
endif(catkin_FOUND)

include_directories(
  include
  ${ZeroMQ_INCLUDE_DIRS}
  ${CAPNP_INCLUDE_DIRS}
)

############# BASIC MESSAGES

## Autogenerate the c++ capnproto files
set(capnp_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE capnp_messages ${capnp_message_folder} *.capnp)

set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/msg)
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero-base-msgs")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

message("CAPNP_SRCS: ${CAPNP_SRCS}")
message("CAPNP_HDRS: ${CAPNP_HDRS}")

## Declare the C++ capnproto message library
add_library(capnzero-base-msgs
  ${CAPNP_SRCS}
)

############ PUBLISH-SUBSCRIBE-LIBRARY

add_library(${PROJECT_NAME}
  src/Publisher.cpp
  src/Subscriber.cpp
        include/capnzero/CapnZero.h)

target_link_libraries(${PROJECT_NAME}
  ${ZeroMQ_LIBRARIES}
  ${CAPNP_LIBRARIES}
)

if (NOT catkin_FOUND)
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

############# SENDER-CONSOLE-APP

add_executable(pub
        src/pub.cpp
)

target_link_libraries(pub
  ${PROJECT_NAME}
  ${ZeroMQ_LIBRARIES}
  ${CAPNP_LIBRARIES}
  capnzero-base-msgs
)

if (NOT catkin_FOUND)
  target_include_directories(pub PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

############# RECEIVER-CONSOLE-APP

add_executable(echo
        src/echo.cpp
)

target_link_libraries(echo
  ${PROJECT_NAME}
  ${ZeroMQ_LIBRARIES}
  ${CAPNP_LIBRARIES}
  capnzero-base-msgs
  pthread
)

if (NOT catkin_FOUND)
  target_include_directories(echo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

