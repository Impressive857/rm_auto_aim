cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖查找
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 获取项目根目录绝对路径
get_filename_component(BASE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

# 文件收集与路径格式化
set(FORMATTED_FILES "")

# 处理msg文件
file(GLOB MSG_FILES "msg/*.msg")
foreach(msg_file ${MSG_FILES})
  file(RELATIVE_PATH REL_PATH "${BASE_PATH}" "${msg_file}")
  list(APPEND FORMATTED_FILES "${BASE_PATH}:${REL_PATH}")
endforeach()

# 处理srv文件
file(GLOB SRV_FILES "srv/*.srv")
foreach(srv_file ${SRV_FILES})
  file(RELATIVE_PATH REL_PATH "${BASE_PATH}" "${srv_file}")
  list(APPEND FORMATTED_FILES "${BASE_PATH}:${REL_PATH}")
endforeach()

# 接口生成（关键修改点）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${FORMATTED_FILES}
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_package()
