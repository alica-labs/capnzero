cmake_minimum_required(VERSION 2.8.3)
project(rosmq)

## Use c++ 11x std
add_definitions(-std=c++11)

# Define where to find modules for ZeroMQ
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(Threads)
find_package(ZeroMQ REQUIRED)

find_package(catkin REQUIRED roscpp)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosmq
  DEPENDS ZeroMQ
  CATKIN_DEPENDS roscpp
  CFG_EXTRAS rosmq-extras.cmake
)

include_directories(
  include
  ${ZeroMQ_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

############ rosmq Communication Library
add_library(${PROJECT_NAME}
  src/Publisher.cpp
  src/Subscriber.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ZMQ_BUILD_DRAFT_API=1)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${ZeroMQ_LIBRARIES}
  ${catkin_LIBRARIES}
)

############# SENDER-CONSOLE-APP

add_executable(pub
  src/pub.cpp
)

target_link_libraries(pub
  ${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)

############# RECEIVER-CONSOLE-APP

add_executable(echo
  src/echo.cpp
)

target_link_libraries(echo
  ${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)

