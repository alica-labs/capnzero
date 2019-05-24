cmake_minimum_required(VERSION 2.8.3)
project(id)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES id
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
)

#### ID MESSAGES
find_package(CapnProto REQUIRED)
set(${PROJECT_NAME}_msgs "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero")
file(GLOB_RECURSE capnp_messages ${${PROJECT_NAME}_msgs} *.capnp)
set(CAPNPC_SRC_PREFIX ${${PROJECT_NAME}_msgs})
set(CAPNPC_OUTPUT_DIR ${${PROJECT_NAME}_msgs})
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

add_library(${PROJECT_NAME}
  ${CAPNP_SRCS}
  ${CAPNP_LIBRARIES}
)
