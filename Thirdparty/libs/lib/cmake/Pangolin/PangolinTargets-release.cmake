#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pangolin" for configuration "Release"
set_property(TARGET pangolin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pangolin PROPERTIES
  IMPORTED_LOCATION_RELEASE "/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM/Thirdparty/Pangolin-0.6/../libs/lib/libpangolin.so"
  IMPORTED_SONAME_RELEASE "libpangolin.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pangolin )
list(APPEND _IMPORT_CHECK_FILES_FOR_pangolin "/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM/Thirdparty/Pangolin-0.6/../libs/lib/libpangolin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
