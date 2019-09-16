cmake_minimum_required(VERSION 2.8.3)
project(capnzero)

## Use c++ 11x std
add_definitions(-std=c++11)

# Define where to find modules for ZeroMQ
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(Threads)
find_package(ZeroMQ REQUIRED)
find_package(CapnProto REQUIRED)

# copying the variables makes it compatible for catkin to transitive forward the CapnProto libraries and include dirs
set(CapnProto_LIBRARIES ${CAPNP_LIBRARIES})
set(CapnProto_INCLUDE_DIRS ${CAPNP_INCLUDE_DIRS})

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES capnzero capnzero-base-msgs
  DEPENDS ZeroMQ CapnProto
  CFG_EXTRAS CapnZero-extras.cmake
)

include_directories(
  include
  ${ZeroMQ_INCLUDE_DIRS}
  ${CAPNP_INCLUDE_DIRS}
)

############ CapnZero Communication Library

add_library(${PROJECT_NAME}
  src/Publisher.cpp
  src/Subscriber.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ZMQ_BUILD_DRAFT_API=1)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${ZeroMQ_LIBRARIES}
  ${CAPNP_LIBRARIES}
)

############# SENDER-CONSOLE-APP

add_executable(pub
  src/pub.cpp
)

target_link_libraries(pub
  ${PROJECT_NAME}
  capnzero-base-msgs
  ${CMAKE_THREAD_LIBS_INIT}
)

############# RECEIVER-CONSOLE-APP

add_executable(echo
  src/echo.cpp
)

target_link_libraries(echo
  ${PROJECT_NAME}
  capnzero-base-msgs
  ${CMAKE_THREAD_LIBS_INIT}
)

############# BASIC MESSAGES

## Autogenerate the c++ capnproto files
set(capnp_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE capnp_messages ${capnp_message_folder} *.capnp)

set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero-base-msgs")
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero-base-msgs")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

add_library(capnzero-base-msgs
  ${CAPNP_SRCS}
)
