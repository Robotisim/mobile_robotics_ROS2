#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_cpp" for configuration "Debug"
set_property(TARGET my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_cpp PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmy_custom_message__rosidl_typesupport_fastrtps_cpp.so"
  IMPORTED_SONAME_DEBUG "libmy_custom_message__rosidl_typesupport_fastrtps_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_custom_message::my_custom_message__rosidl_typesupport_fastrtps_cpp "${_IMPORT_PREFIX}/lib/libmy_custom_message__rosidl_typesupport_fastrtps_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
