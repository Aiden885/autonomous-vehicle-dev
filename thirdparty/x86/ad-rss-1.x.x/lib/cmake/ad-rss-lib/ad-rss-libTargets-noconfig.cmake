#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ad-rss-lib" for configuration ""
set_property(TARGET ad-rss-lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ad-rss-lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libad-rss.so.1.4.0"
  IMPORTED_SONAME_NOCONFIG "libad-rss.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ad-rss-lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_ad-rss-lib "${_IMPORT_PREFIX}/lib/libad-rss.so.1.4.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
