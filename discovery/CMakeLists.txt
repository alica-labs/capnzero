cmake_minimum_required(VERSION 2.8.3)
project(discovery)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
        set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
        set(CMAKE_ECLIPSE_VERSION "4.6" CACHE STRING "Eclipse version" FORCE)
        add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Define where to find modules for ZeroMQ and UUID

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

# this enables the draft part of the api
add_definitions(-DZMQ_BUILD_DRAFT_API=1 -DENABLE_DRAFTS=On)
find_package(ZeroMQ REQUIRED)
#message("############################ ${PROJECT_NAME}: ZMQ-Includes: ${ZeroMQ_INCLUDE_DIRS}")
#message("############################ ${PROJECT_NAME}: ZMQ-Libraries: ${ZeroMQ_LIBRARIES}")

find_package(UUID)
#message("${PROJECT_NAME}: UUID-Includes: ${UUID_INCLUDE_DIRS}") 
#message("${PROJECT_NAME}: UUID-Libraries: ${UUID_LIBRARIES}")


# Introduce CapnProto
find_package(CapnProto REQUIRED)


# Catkin Stuff
find_package(catkin QUIET)
if (catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS event_handling discovery_msgs capnzero)
else (catkin_FOUND)
  set(catkin_LIBRARIES "event_handling discovery_msgs capnzero")
endif(catkin_FOUND)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
  # LIBRARIES 
    CATKIN_DEPENDS event_handling discovery_msgs capnzero
    DEPENDS ZeroMQ UUID
  )
endif(catkin_FOUND)

#message("${PROJECT_NAME}: catkin_LIBRARIES: ${catkin_LIBRARIES}")

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${ZeroMQ_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

#message("${PROJECT_NAME}: catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

## DISCOVERY
add_executable(${PROJECT_NAME}
  src/Discovery.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${ZeroMQ_LIBRARIES}
)

if (NOT catkin_FOUND)
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

## AGENT
add_executable(Agent
  src/Agent.cpp
)


target_link_libraries(Agent
  ${catkin_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
  ${UUID_LIBRARIES}
  ${CAPNP_LIBRARIES}
  pthread
)

## SEND TEST
add_executable(SendTest
  src/SendTest.cpp
)

target_link_libraries(SendTest
  ${catkin_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
  ${UUID_LIBRARIES}
  ${CAPNP_LIBRARIES}
  pthread
)

if (NOT catkin_FOUND)
 target_include_directories(Agent PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)
