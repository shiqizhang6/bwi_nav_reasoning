cmake_minimum_required(VERSION 2.8.3)
project(bwi_nav_reasoning)

find_package(catkin REQUIRED 
    COMPONENTS 
        roslib 
        roscpp 
        std_msgs 
        message_generation
        bwi_rl
        bwi_msgs
)
find_package(Boost REQUIRED 
    COMPONENTS 
        serialization
)

generate_messages(
    DEPENDENCIES 
        bwi_msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS 
        roslib 
        roscpp 
        bwi_rl 
        bwi_msgs 
        message_runtime
)

include_directories(
    src
    ${bwi_rl_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


add_executable(bwi_nav_reasoning_test 
    src/test.cpp 
    src/DomainParser.cpp 
    src/Driver.cpp
    src/StateAction.cpp
)

target_link_libraries(bwi_nav_reasoning_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)
