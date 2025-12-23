cmake_minimum_required(VERSION 3.20)
project(visualization)

include_directories(include)
include_directories(../parameter/include)
include_directories(../clock/include)

include_directories(/opt/ros/humble/include/image_transport)
include_directories(/opt/ros/humble/include/cv_bridge)
include_directories(/opt/ros/humble/include/sensor_msgs)

list(APPEND Dependencies
        rclcpp
        rcl
        rosidl_runtime_c
        rm_utils
        cv_bridge
        image_transport
        sensor_msgs
        std_msgs
        visualization_msgs
        rm_interfaces
        tf2_ros
        tf2_geometry_msgs
)

foreach (dep_ ${Dependencies})
    find_package(${dep_} REQUIRED)
endforeach ()

foreach (_dep ${Dependencies})
    include_directories(${${_dep}_INCLUDE_DIRS})
endforeach ()

ament_auto_add_library(Visualization SHARED src/visualization.cpp)
list(APPEND Libraries Visualization)

#show_details(${PROJECT_NAME} "${Dependencies}" "${Libraries}")
export_submodule("${Libraries}")
export_include(${PROJECT_SOURCE_DIR}/include)
