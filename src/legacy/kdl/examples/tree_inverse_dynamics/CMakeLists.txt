cmake_minimum_required(VERSION 2.8.7)

# Start a project.
project(tree_inverse_dynamics)

#find orocos_kdl, with some glue code to support also the use of the old version
find_package(orocos_kdl 1.2.3 QUIET)
if (NOT orocos_kdl_FOUND)
    SET(OROCOS_KDL_OLDVERSION On)
    find_package(orocos_kdl)
endif(NOT orocos_kdl_FOUND)

#support also for the old version of kdl cmake package
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


# Search for source code.
file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# Automatically add include directories if needed.
foreach(header_file ${folder_header})
  get_filename_component(p ${header_file} PATH)
  include_directories(${p})
endforeach(header_file ${folder_header})


# Set up our main executable.
if (folder_source)
  add_executable(tree_inverse_dynamics ${folder_source} ${folder_header})
  target_link_libraries(tree_inverse_dynamics ${orocos_kdl_LIBRARIES})
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)
