cmake_minimum_required(VERSION 3.0.2)
project(cv_06)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        image_transport
        sensor_msgs
        tf
        dynamic_reconfigure
        nodelet
        pluginlib
        )                         

generate_dynamic_reconfigure_options(
        cfg/dynamic.cfg
)

find_package(OpenCV REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)
add_library(main6 src/main6.cpp)
#add_executable(main6 src/main6.cpp)
add_dependencies(main6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})