cmake_minimum_required(VERSION 3.10)
project(ChassisControl VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 添加DLL导出定义
add_definitions(-DCHASSIS_CONTROL_EXPORTS)

# 构建类型，如果没有指定则使用Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
endif()

# 生成动态库
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# 设置库版本
set(CHASSIS_CONTROL_VERSION_MAJOR 1)
set(CHASSIS_CONTROL_VERSION_MINOR 0)
set(CHASSIS_CONTROL_VERSION_PATCH 0)
set(CHASSIS_CONTROL_VERSION "${CHASSIS_CONTROL_VERSION_MAJOR}.${CHASSIS_CONTROL_VERSION_MINOR}.${CHASSIS_CONTROL_VERSION_PATCH}")

# 设置包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/Vehicle_Data_Input
)

# 添加数据输入模块源文件
set(DATA_INPUT_SOURCES
  ${PROJECT_SOURCE_DIR}/src/Vehicle_Data_Input/VehicleDataInput.cpp
)

# 添加所有源文件到一个列表中
set(ALL_SOURCES
  ${DATA_INPUT_SOURCES}
)

# 创建单一的动态链接库，包含所有模块
add_library(chassis_control ${ALL_SOURCES})

# 设置库属性
set_target_properties(chassis_control PROPERTIES
    VERSION ${CHASSIS_CONTROL_VERSION}
    SOVERSION ${CHASSIS_CONTROL_VERSION_MAJOR}
    PUBLIC_HEADER "${PROJECT_SOURCE_DIR}/chassis_control.h"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 设置库的包含目录
target_include_directories(chassis_control
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

# 示例程序
add_executable(basic_control_example ${CMAKE_SOURCE_DIR}/examples/basic_control_example.cpp)
target_link_libraries(basic_control_example
    PRIVATE
    chassis_control
)

target_include_directories(basic_control_example
    PRIVATE
    ${PROJECT_SOURCE_DIR}
)

# 设置示例程序的输出目录
set_target_properties(basic_control_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 自动复制DLL到示例程序目录
add_custom_command(TARGET basic_control_example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:chassis_control>
    $<TARGET_FILE_DIR:basic_control_example>
)

# 安装头文件（只安装公共接口头文件）
install(FILES chassis_control.h DESTINATION include)

# 生成并安装导出目标
install(TARGETS chassis_control basic_control_example
    EXPORT chassis_control-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# 创建并安装版本配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/chassis_control-config-version.cmake"
  VERSION ${CHASSIS_CONTROL_VERSION}
  COMPATIBILITY SameMajorVersion
)

# 安装导出目标
install(EXPORT chassis_control-targets
  FILE chassis_control-targets.cmake
  DESTINATION lib/cmake/chassis_control
)

# 创建配置文件
configure_file(cmake/chassis_control-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/chassis_control-config.cmake"
  @ONLY
)

# 安装配置文件
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/chassis_control-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/chassis_control-config-version.cmake"
  DESTINATION lib/cmake/chassis_control
)

# 打印配置信息
message(STATUS "ChassisControl ${CHASSIS_CONTROL_VERSION} build configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "  Shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}") 