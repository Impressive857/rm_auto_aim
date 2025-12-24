cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## 基础编译配置（不变）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## 依赖配置（精简G2O路径，不硬编码绝对文件）
#######################
# 1. G2O配置（关键修改：只指定目录，不写具体.so路径）
# 适配两种常见安装路径：手动编译（/usr/local）、系统预编译（/usr）
set(G2O_INCLUDE_DIRS
        /usr/local/include/g2o  # 手动编译G2O的头文件路径（优先）
        /usr/include/g2o        # 系统预编译G2O的头文件路径（备选）
)
set(G2O_LIBRARY_DIRS
        /usr/local/lib          # 手动编译G2O的库路径（优先）
        /usr/lib/x86_64-linux-gnu  # 系统预编译G2O的库路径（备选）
)
# G2O库名（只写库名，CMake从上面的目录自动找）
set(G2O_LIBRARIES
        g2o_core
        g2o_stuff
        g2o_solver_csparse
        g2o_types_sba
        g2o_types_slam3d
        g2o_solver_dense
        g2o_csparse_extension
)

# 2. 其他依赖（原配置不变，保留原生查找逻辑）
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
ament_auto_find_build_dependencies()

# 3. 头文件+库目录（合并配置，不冗余）
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
        ${G2O_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
)

###########
## 编译+链接（不变）
###########
ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${G2O_LIBRARIES}  # 用库名链接，自动匹配上面的目录
        fmt::fmt
        tbb
        Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN fyt::auto_aim::ArmorDetectorNode
        EXECUTABLE armor_detector_node
)

#############
## 测试+安装（不变）
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
          ament_cmake_copyright
          ament_cmake_uncrustify
          ament_cmake_cpplint
          ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        docs
        model
)
