cmake_minimum_required(VERSION 3.10)
project(armor_solver)

#set(CMAKE_BUILD_TYPE Debug)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Wextra -Wpedantic)

add_definitions(-DBUILD_DIR="${PROJECT_BINARY_DIR}")
add_definitions(-DINDEX_NAME="registered_index.hpp")
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
# 添加Eigen3的查找
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include executor ${async_frame_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})

#######################
##   Build target    ##
#######################
ament_auto_add_library(KalmanPool
        src/kalman_pool/kalman_pool.cpp
        src/kalman_pool/object_base.cpp
        src/kalman_pool/observer.cpp
        src/kalman_pool/outpost.cpp
        src/kalman_pool/robot.cpp)
list(APPEND LIBS KalmanPool)

ament_auto_add_library(GimbalController DIRECTORY src/gimbal_controller)
list(APPEND LIBS GimbalController)

# Add tinympcstatic library
add_library(tinympcstatic STATIC
        include/armor_solver/tinympc/admm.cpp
        include/armor_solver/tinympc/tiny_api.cpp
        include/armor_solver/tinympc/codegen.cpp
        include/armor_solver/tinympc/rho_benchmark.cpp
)

set_property(TARGET tinympcstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# 修正include路径
target_include_directories(tinympcstatic PUBLIC
        include/armor_solver/tinympc
        ${EIGEN3_INCLUDE_DIR}
)

# 使用Eigen3的现代CMake方式
target_compile_definitions(tinympcstatic PRIVATE EIGEN_MPL2_ONLY)
target_link_libraries(tinympcstatic PUBLIC Eigen3::Eigen)

# Add tinympcstatic to libraries list
list(APPEND LIBS tinympcstatic)

ament_auto_add_executable(${PROJECT_NAME}_node executor/executor.cpp)

#######################
## Compiled submodule##
#######################

include(submodule/cmake/Function.cmake)
list(APPEND Sub_Library)
list(APPEND Sub_Include)

add_subdirectory(submodule/parameter)
add_subdirectory(submodule/visualization)
add_subdirectory(submodule/clock)
add_subdirectory(submodule/data_center)
message(STATUS "Sub Module Contains: ${Sub_Library}")
include_directories(${Sub_Include})

foreach (LIB ${LIBS})
    link_libraries(${LIB} ${Sub_Library})
endforeach ()

target_link_libraries(${PROJECT_NAME}_node
        ${Sub_Library}
        ${LIBS}
        ${async_frame_LIBRARIES}
        Eigen3::Eigen)

###########################
## Codegen configuration ##
###########################

if(USING_CODEGEN) # Defined in top-level CMakeLists.txt

    # Files that are needed for embedded code generation
    list( APPEND EMBEDDED_FILES
            "include/armor_solver/tinympc/admm.cpp"
            "include/armor_solver/tinympc/admm.hpp"
            "include/armor_solver/tinympc/tiny_api.cpp"
            "include/armor_solver/tinympc/tiny_api.hpp"
            "include/armor_solver/tinympc/types.hpp"
            "include/armor_solver/tinympc/tiny_api_constants.hpp" )


    foreach( f ${EMBEDDED_FILES} )
        get_filename_component( fname ${f} NAME )
        set( dest_file "${EMBEDDED_BUILD_TINYMPC_DIR}/${fname}" )
        list( APPEND EMBEDDED_BUILD_TINYMPC_FILES "${dest_file}" )

        add_custom_command(OUTPUT ${dest_file}
                COMMAND ${CMAKE_COMMAND} -E copy "${f}" "${dest_file}"
                DEPENDS ${f}
                COMMENT "Copying ${fname}")
    endforeach()

    add_custom_target( copy_codegen_tinympc_files DEPENDS ${EMBEDDED_BUILD_TINYMPC_FILES} )
    add_dependencies( copy_codegen_files copy_codegen_tinympc_files )

endif(USING_CODEGEN)

#######################
## Testing and Debug ##
#######################

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    #    add_definitions(-DDEBUG_MODE)
    file(GLOB_RECURSE AllSrc src/*.cpp executor/*.cpp)
    ament_auto_add_executable(Test_${PROJECT_NAME} ${AllSrc})
    target_link_libraries(Test_${PROJECT_NAME} ${Sub_Library} ${async_frame_LIBRARIES} Eigen3::Eigen)
endif ()

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
            ament_cmake_copyright
            ament_cmake_uncrustify
            ament_cmake_cpplint
    )
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)

endif ()

#############
## Install ##
#############
install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY executor/
        DESTINATION include/executor
        PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}_node
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(TARGETS tinympcstatic
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
)

#export include
ament_export_include_directories("include")

#export lib
ament_export_libraries(${LIBS} ${Sub_Library})

ament_auto_package()