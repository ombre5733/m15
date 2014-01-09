include(CMakeParseArguments)

cmake_parse_arguments(_args "" "TARGET;SOURCE_LIST" "" ${SUBMODULE_OPTIONS})

if (NOT "${_args_UNPARSED_ARGUMENTS}" STREQUAL "")
    message(FATAL_ERROR "m15-usb: Unknown options '${_args_UNPARSED_ARGUMENTS}'.")
endif()

# The source files which are necessary for this module.
set(_sources
        "${M15_SOURCE_DIR}/usb/usbdevicecoredriver.cpp")

# If a TARGET is specified, we create a static library from the module's
# sources and link with it.
if(_args_TARGET)
    set(_library_name "m15-usb-${_args_TARGET}")
    add_library(${_library_name} STATIC ${_sources})
    target_link_libraries(${_args_TARGET} ${_library_name})
endif()

# If a SOURCE_LIST is specified, the module's sources are added to it.
if(_args_SOURCE_LIST)
    set(_new_list ${${_args_SOURCE_LIST}})
    list(APPEND _new_list ${_sources})
    set(${_args_SOURCE_LIST} ${_new_list} PARENT_SCOPE)
endif()
