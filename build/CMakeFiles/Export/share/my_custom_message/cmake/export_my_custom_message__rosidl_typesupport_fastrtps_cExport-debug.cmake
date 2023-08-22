#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_c" for configuration "Debug"
set_property(TARGET my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_c PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmy_custom_message__rosidl_typesupport_fastrtps_c.so"
  IMPORTED_SONAME_DEBUG "libmy_custom_message__rosidl_typesupport_fastrtps_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_c "${_IMPORT_PREFIX}/lib/libmy_custom_message__rosidl_typesupport_fastrtps_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
