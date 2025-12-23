cmake_minimum_required(VERSION 3.20)
project(parameter)

include_directories(include)

list(APPEND Dependencies
        rclcpp
        rcl
        rosidl_runtime_c
        rm_utils
)

foreach (_dep ${Dependencies})
    include_directories(${${_dep}_INCLUDE_DIRS})
endforeach ()

add_library(Parameter SHARED src/parameter.cpp)
list(APPEND Libraries Parameter)

#show_details(${PROJECT_NAME} "${Dependencies}" "${Libraries}")
export_submodule("${Libraries}")
export_include(${PROJECT_SOURCE_DIR}/include)