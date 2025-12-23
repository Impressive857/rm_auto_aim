cmake_minimum_required(VERSION 3.20)
project(data_center)

include_directories(include)
include_directories(../clock/include)
include_directories(../parameter/include)
include_directories(../visualization/include)

include_directories(/opt/ros/humble/include/image_transport)
include_directories(/opt/ros/humble/include/cv_bridge)

list(APPEND Dependencies
        rclcpp
        rcl
        rosidl_runtime_c
        tf2_ros
        rm_interfaces
        std_msgs
        geometry_msgs
        rm_utils
)

foreach (_dep ${Dependencies})
    include_directories(${${_dep}_INCLUDE_DIRS})
endforeach ()

add_library(ROS_Listener SHARED src/listener/ros_listener.cpp)
list(APPEND Libraries ROS_Listener)

add_library(DataRecorder SHARED src/data_recorder.cpp)
list(APPEND Libraries DataRecorder)

foreach (_lib ${Libraries})
    foreach (_dep ${Dependencies})
        link_libraries(${_lib} ${${_dep}_LIBRARIES})
    endforeach ()
endforeach ()

#show_details(${PROJECT_NAME} "${Dependencies}" "${Libraries}")
export_submodule("${Libraries}")
export_include(${PROJECT_SOURCE_DIR}/include)