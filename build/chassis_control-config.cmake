

include(CMakeFindDependencyMacro)
find_dependency(yaml-cpp REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/chassis_control-targets.cmake") 
