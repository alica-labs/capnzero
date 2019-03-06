cmake_minimum_required(VERSION 2.8.3)
project(capnproto_performance_test)


## Use c++ 11x std
add_definitions(-std=c++11)
#added by me
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
# Define where to find modules for ZeroMQ and UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)


# this enables the draft part of the api
#add_definitions(-DZMQ_BUILD_DRAFT_API=1)

#ZMQ
find_package(ZeroMQ)

# CapnProto
find_package(CapnProto REQUIRED)

find_package(catkin REQUIRED COMPONENTS capnzero)

message("CapnProto Incl: ${CAPNP_INCLUDE_DIRS}")
message("CapnProto Lib: ${CAPNP_LIBRARIES}")

find_package(catkin QUIET)
if(catkin_FOUND)
    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES capnproto_performance_test capnproto-base-msgs
            DEPENDS ZeroMQ CapnProto
            CATKIN_DEPENDS capnzero
    )
    else (catkin_FOUND)
    set(catkin_LIBRARIES capnzero)
endif(catkin_FOUND)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${ZeroMQ_INCLUDE_DIRS}
        ${CAPNP_INCLUDE_DIRS}
        ${capnzero_INCLUDE_DIRS}
)

############# BASIC MESSAGES
## Autogenerate the c++ capnproto files
set(capnproto_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE capnproto_messages ${capnp_message_folder} *.capnp)

set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/msg)
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/capnproto-base-msgs")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnproto_messages})

message("CAPNP_SRCS: ${CAPNP_SRCS}")
message("CAPNP_HDRS: ${CAPNP_HDRS}")

## Declare the C++ capnproto message library
add_library(capnproto-base-msgs
        ${CAPNP_SRCS}
        )

############ PUBLISH-SUBSCRIBE-LIBRARY

if (NOT catkin_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

############# SENDER-CONSOLE-APP

add_executable(capnzeroPerformanceTest
        src/capnzeroPerformancetest.cpp src/Statistics.cpp
        )

target_link_libraries(capnzeroPerformanceTest
        capnproto-base-msgs
        ${ZeroMQ_LIBRARIES}
        ${CAPNP_LIBRARIES}
        ${catkin_LIBRARIES}
        )

target_compile_definitions(capnzeroPerformanceTest PUBLIC ZMQ_BUILD_DRAFT_API=1)

############# RECEIVER-CONSOLE-APP

add_executable(Parrot
        src/Parrot.cpp
        )

target_link_libraries(Parrot
        ${ZeroMQ_LIBRARIES}
        ${CAPNP_LIBRARIES}
        ${catkin_LIBRARIES}
        capnproto-base-msgs
        pthread
        )

target_compile_definitions(Parrot PUBLIC ZMQ_BUILD_DRAFT_API=1)

