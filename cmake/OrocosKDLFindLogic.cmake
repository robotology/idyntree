#########################################################################
find_package(orocos_kdl 1.2.3 QUIET)
if(NOT orocos_kdl_FOUND)
    SET(OROCOS_KDL_OLDVERSION On)
    find_package(orocos_kdl)
else()
    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        find_package(Boost REQUIRED)
    endif()
endif(NOT orocos_kdl_FOUND)

#support also for the old version of kdl cmake package
#mostly for people getting orocos_kdl from old ROS versions
if(NOT orocos_kdl_FOUND)
   find_package(Orocos-KDL)
   if(NOT Orocos-KDL_FOUND)
      message(WARNING "KDL not found: neither orocos_kdl or Orocos-KDL cmake packages are available")
   else(NOT Orocos-KDL_FOUND)
      set(orocos_kdl_INCLUDE_DIRS ${Orocos-KDL_INCLUDE_DIRS})
      set(orocos_kdl_LIBRARY_DIRS ${Orocos-KDL_LIBRARY_DIRS})
      set(orocos_kdl_LIBRARIES ${Orocos-KDL_LIBRARIES})
      set(orocos_kdl_FOUND true)
      set(orocos_kdl_VERSION ${Orocos-KDL_VERSION})
      set(orocos_kdl_VERSION_MAJOR ${Orocos-KDL_VERSION_MAJOR})
      set(orocos_kdl_VERSION_MINOR ${Orocos-KDL_VERSION_MINOR})
      set(orocos_kdl_VERSION_PATCH ${Orocos-KDL_VERSION_PATCH})
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

# Add OROCOS_KDL version definitions on the code
add_definitions(-DOROCOS_KDL_VERSION_MAJOR=${orocos_kdl_VERSION_MAJOR})
add_definitions(-DOROCOS_KDL_VERSION_MINOR=${orocos_kdl_VERSION_MINOR})
add_definitions(-DOROCOS_KDL_VERSION_PATCH=${orocos_kdl_VERSION_PATCH})

