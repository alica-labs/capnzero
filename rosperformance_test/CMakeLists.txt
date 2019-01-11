cmake_minimum_required(VERSION 2.8.3)
project(rosperformance_test)



find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs )

## Generate messages in the 'msg' folder
add_message_files(FILES Msgs.msg)

generate_messages(DEPENDENCIES std_msgs)
##generate_messages(DEPENDENCIES std_msgs)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
##generate_messages(DEPENDENCIES std_msgs)
catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp std_msgs message_runtime
)
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(parrot src/Parrot_subcriber.cpp)
target_link_libraries(parrot ${catkin_LIBRARIES})

add_dependencies(parrot rosperformance_test_generate_messages_cpp)

add_executable(Rospcommunication src/ROSPerformanceTest.cpp src/Statistics.cpp)
target_link_libraries(Rospcommunication   ${catkin_LIBRARIES})
