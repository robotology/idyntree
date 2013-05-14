if (kdl_codyco_CONFIG_INCLUDED)
  return()
endif()
set(kdl_codyco_CONFIG_INCLUDED TRUE)

set(kdl_codyco_INCLUDE_DIRS /usr/local/include)

foreach(lib kdl_codyco)
  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
    )
  if(NOT onelib)
    message(FATAL_ERROR "Library '${lib}' in package kdl_codyco is not installed properly")
  endif()
  list(APPEND kdl_codyco_LIBRARIES ${onelib})
endforeach()

foreach(dep )
  if(NOT ${dep}_FOUND)
    find_package(${dep})
  endif()
  list(APPEND kdl_codyco_INCLUDE_DIRS ${${dep}_INCLUDE_DIRS})
  list(APPEND kdl_codyco_LIBRARIES ${${dep}_LIBRARIES})
endforeach()
