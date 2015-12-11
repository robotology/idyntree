# Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
# Author: Ali Paikan <ali.paikan@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


# TinyXML
set(PROJECTNAME iDynTree_priv_tinyxml)
project(${PROJECTNAME})

set(tinyxml_SRCS
    src/tinyxml.cpp
    src/tinyxmlerror.cpp
    src/tinyxmlparser.cpp
)

set(tinyxml_HDRS
    src/tinystr.h
    src/tinyxml.h
)

add_definitions("-DTIXML_USE_STL")
add_library(${PROJECTNAME} STATIC ${tinyxml_SRCS})

set(TinyXML_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src PARENT_SCOPE)
set(TinyXML_LIBRARIES "${PROJECTNAME}" PARENT_SCOPE)
set(TinyXML_DEFINITIONS "-DTIXML_USE_STL " PARENT_SCOPE)

install(TARGETS ${PROJECTNAME}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})

