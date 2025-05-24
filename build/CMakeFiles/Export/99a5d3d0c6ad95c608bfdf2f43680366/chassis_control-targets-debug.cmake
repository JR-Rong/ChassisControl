#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "chassis_control" for configuration "Debug"
set_property(TARGET chassis_control APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(chassis_control PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/libchassis_control.dll.a"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/libchassis_control.dll"
  )

list(APPEND _cmake_import_check_targets chassis_control )
list(APPEND _cmake_import_check_files_for_chassis_control "${_IMPORT_PREFIX}/lib/libchassis_control.dll.a" "${_IMPORT_PREFIX}/bin/libchassis_control.dll" )

# Import target "basic_control_example" for configuration "Debug"
set_property(TARGET basic_control_example APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(basic_control_example PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/basic_control_example.exe"
  )

list(APPEND _cmake_import_check_targets basic_control_example )
list(APPEND _cmake_import_check_files_for_basic_control_example "${_IMPORT_PREFIX}/bin/basic_control_example.exe" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
