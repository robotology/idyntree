cmake_minimum_required(VERSION 2.8.11)

# Start a project.
project(subtree_base_regressor)

#find orocos_kdl, with some glue code to support also the use of the old version
find_package(orocos_kdl 1.2.3 REQUIRED QUIET)

find_package(YCM REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YCM_MODULE_PATH})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)

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

# Include any directories needed
include_directories(${kdl_format_io_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})

# Set up our main executable.
if (folder_source)
  add_executable(subtree_base_regressor ${folder_source} ${folder_header})
  target_link_libraries(subtree_base_regressor ${orocos_kdl_LIBRARIES}
                                               ${kdl_codyco_LIBRARIES}
                                                ${kdl_format_io_LIBRARIES}
                                                 ${YARP_LIBRARIES} ${ICUB_LIBRARIES}
                                                ${iDynTree_LIBRARIES})
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)
