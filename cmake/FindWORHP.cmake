include(FindPackageHandleStandardArgs)

find_path(WORHP_INCLUDE_DIR worhp HINTS ENV WORHP_DIR)
mark_as_advanced(WORHP_INCLUDE_DIR)
find_library(WORHP_LIBRARY worhp HINTS ENV WORHP_DIR)
mark_as_advanced(WORHP_LIBRARY)

find_package_handle_standard_args(WORHP DEFAULT_MSG WORHP_INCLUDE_DIR WORHP_LIBRARY)

if(WORHP_FOUND)
    set(WORHP_INCLUDE_DIRS ${WORHP_INCLUDE_DIR})
    set(WORHP_LIBRARIES ${WORHP_LIBRARY})

    if(NOT TARGET WORHP::WORHP)
      add_library(WORHP::WORHP UNKNOWN IMPORTED)
      set_target_properties(WORHP::WORHP PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${WORHP_INCLUDE_DIR}")
      set_property(TARGET WORHP::WORHP APPEND PROPERTY
        IMPORTED_LOCATION "${WORHP_LIBRARY}")
    endif()
endif()
