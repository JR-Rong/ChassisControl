cmake_minimum_required(VERSION 3.10)
project(ChassisControl VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# 查找依赖包
find_package(yaml-cpp REQUIRED)

# 设置包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${YAML_CPP_INCLUDE_DIR}
)

# 添加数据输入模块源文件
set(DATA_INPUT_SOURCES
  src/data_input/vehicle_config.cpp
  src/data_input/state_estimator.cpp
)

# 添加车辆控制模块源文件
set(VEHICLE_CONTROL_SOURCES
  src/vehicle_control/vehicle_model.cpp
  src/vehicle_control/torque_allocator.cpp
  src/vehicle_control/torque_smoother.cpp
  src/vehicle_control/tire_model.cpp
)

# 添加数据输出模块源文件
set(DATA_OUTPUT_SOURCES
  src/data_output/can_bus_interface.cpp
)

# 添加高层接口源文件
set(API_SOURCES
  src/chassis_control.cpp
)

# 添加所有源文件到一个列表中
set(ALL_SOURCES
  ${DATA_INPUT_SOURCES}
  ${VEHICLE_CONTROL_SOURCES}
  ${DATA_OUTPUT_SOURCES}
  ${API_SOURCES}
)

# 创建单一的动态链接库，包含所有模块
add_library(chassis_control ${ALL_SOURCES})
set_target_properties(chassis_control PROPERTIES
  VERSION ${CHASSIS_CONTROL_VERSION}
  SOVERSION ${CHASSIS_CONTROL_VERSION_MAJOR}
  PUBLIC_HEADER "include/chassis_control.h"
)

# 链接依赖库
target_link_libraries(chassis_control
  ${YAML_CPP_LIBRARIES}
  pthread
)

# 示例程序
add_executable(basic_control_example examples/basic_control_example.cpp)
target_link_libraries(basic_control_example
    chassis_control
    ${YAML_CPP_LIBRARIES}
    pthread
)

# API示例程序
add_executable(simple_api_example examples/simple_api_example.cpp)
target_link_libraries(simple_api_example
    chassis_control
    ${YAML_CPP_LIBRARIES}
    pthread
)

# 生成并安装导出目标
install(TARGETS chassis_control
  EXPORT chassis_control-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include
)

# 安装头文件（只安装公共接口头文件）
install(FILES include/chassis_control.h DESTINATION include)

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

# 安装可执行文件
install(TARGETS basic_control_example simple_api_example DESTINATION bin)

# 打印配置信息
message(STATUS "ChassisControl ${CHASSIS_CONTROL_VERSION} build configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "  Shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}") 