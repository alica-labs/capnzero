cmake_minimum_required(VERSION 2.8.3)
project(rosperformance_test)


find_package(catkin REQUIRED COMPONENTS  roscpp std_msgs )

##generate_messages(DEPENDENCIES std_msgs)
catkin_package(
        #INCLUDE_DIRS include
        LIBRARIES rosperformace_test
        CATKIN_DEPENDS roscpp std_msgs
        DEPENDS roscpp
)
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(parrot src/Parrot_subcriber.cpp)
target_link_libraries(parrot ${catkin_LIBRARIES})

add_executable(Rospcommunication src/ROSPerformanceTest.cpp)
target_link_libraries(Rospcommunication   ${catkin_LIBRARIES})
