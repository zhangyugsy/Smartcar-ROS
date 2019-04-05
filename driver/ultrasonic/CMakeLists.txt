cmake_minimum_required(VERSION 2.8.3)
project(ultrasonic)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
        std_msgs
        roscpp)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ultrasonic
 CATKIN_DEPENDS std_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(ultrasonic_single
               src/ultrasonic-single.cpp
               src/jetsonGPIO.cpp
               include/jetsonGPIO.h)

target_link_libraries(ultrasonic_single
            ${catkin_LIBRARIES})

target_include_directories(ultrasonic_single PRIVATE
            ${catkin_INCLUDE_DIRS})

add_dependencies(ultrasonic_single ${catkin_EXPORTED_TARGETS})

add_executable(ultrasonic_dual
               src/ultrasonic-dual.cpp
               src/jetsonGPIO.cpp
               include/jetsonGPIO.h)

target_link_libraries(ultrasonic_dual
            ${catkin_LIBRARIES})

target_include_directories(ultrasonic_dual PRIVATE
            ${catkin_INCLUDE_DIRS})

add_dependencies(ultrasonic_dual ${catkin_EXPORTED_TARGETS})