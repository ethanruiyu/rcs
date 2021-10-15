#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "GlobalPlanner::global_planner" for configuration "Release"
set_property(TARGET GlobalPlanner::global_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(GlobalPlanner::global_planner PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/GlobalPlanner/lib/libglobal_planner.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS GlobalPlanner::global_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_GlobalPlanner::global_planner "${_IMPORT_PREFIX}/GlobalPlanner/lib/libglobal_planner.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
