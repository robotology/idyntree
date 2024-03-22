# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(IDYNTREE_MODELIO_XML_HEADERS include/iDynTree/XMLParser.h
                                 include/iDynTree/XMLElement.h
                                 include/iDynTree/XMLAttribute.h
                                 include/iDynTree/XMLDocument.h)

set(IDYNTREE_MODELIO_XML_PRIVATE_HEADERS )

set(IDYNTREE_MODELIO_XML_SOURCES src/XMLParser.cpp
                                 src/XMLElement.cpp
                                 src/XMLAttribute.cpp
                                 src/XMLDocument.cpp)



SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_XML_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_XML_HEADERS})
SOURCE_GROUP("Private Header Files" FILES ${IDYNTREE_MODELIO_XML_PRIVATE_HEADERS})

# share headers with all iDynTree targets
set(libraryname idyntree-modelio-xml)

add_library(${libraryname} ${IDYNTREE_MODELIO_XML_SOURCES} ${IDYNTREE_MODELIO_XML_HEADERS} ${IDYNTREE_MODELIO_XML_PRIVATE_HEADERS})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

# Test if this works:
# We want to include the same-library header files directly (in the implementation), but with the full prefix in the .h (as this will be public)
target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/iDynTree>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PRIVATE Eigen3::Eigen LibXml2::LibXml2 idyntree-core)


if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS} ${LIBXML2_DEFINITIONS})
endif()

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_XML_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(BUILD_TESTING)
    add_subdirectory(tests)
endif(BUILD_TESTING)
