cmake_minimum_required(VERSION 2.8.3)
project(commad_capnzero)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  capnzero
  id_capnzero
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS capnzero id_capnzero 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#### Command MESSAGES
find_package(CapnProto REQUIRED)
set(CAPNPC_IMPORT_DIRS "${id_capnzero_SOURCE_PREFIX}/include")# include external capnp message schema files
set(${PROJECT_NAME}_msgs "${CMAKE_CURRENT_SOURCE_DIR}/include/capnzero")
file(GLOB_RECURSE capnp_messages ${${PROJECT_NAME}_msgs} *.capnp)
set(CAPNPC_SRC_PREFIX ${${PROJECT_NAME}_msgs})
set(CAPNPC_OUTPUT_DIR ${${PROJECT_NAME}_msgs})
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

add_library(${PROJECT_NAME}
  ${CAPNP_SRCS}
  ${CAPNP_LIBRARIES}
)
