# Copyright: (C) 2013 RobotCub Consortium
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required( VERSION 2.8 FATAL_ERROR )


SET(PROJECTNAME iDynTree)

PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

SET(folder_source src/DynTree.cpp
                  src/yarp_kdl.cpp
                  src/iCubTree.cpp
                  src/iDyn2KDL.cpp
                  src/idyn2kdl_icub.cpp)

SET(folder_header include/iCub/iDynTree/DynTree.h
                  include/iCub/iDynTree/yarp_kdl.h
                  include/iCub/iDynTree/iDyn2KDL.h
                  include/iCub/iDynTree/iCubTree.h
                  include/iCub/iDynTree/idyn2kdl_icub.h)
                  
if(CODYCO_USES_URDFDOM)
     find_package(Boost REQUIRED)
     INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
endif()
        
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(include
                    ${skinDynLib_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIR}
                    ${kdl_format_io_INCLUDE_DIRS})
                    
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     ${ICUB_LIBRARIES}
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     ${kdl_codyco_LIBRARIES}
                                     ${kdl_format_io_LIBRARIES}
				                     ${Boost_LIBRARIES})


icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/iDynTree
                                   FILES ${folder_header})
                                  


message(STATUS "Configuration successful. Type make to compile iDynTree")

