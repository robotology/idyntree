# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ModelIO_URDF CXX)

# for now the iDynTree_ModelIO_URDF support
# is compiled in two different way depending
# if KDL is available or not. If KDL
# is not available, the only parser available
# will be the model one.

if(IDYNTREE_USES_KDL)
    if( IDYNTREE_USES_INTERNAL_URDFDOM )
        # urdfdom not available in the system, using the internal version
        add_definitions(-DURDF_USE_PLAIN_POINTERS)
        add_subdirectory(external/urdfdom)
        set(urdfdom_LIBRARIES iDynTree_priv_urdfdom_model)
    else()
        find_package(urdfdom_headers REQUIRED)
        find_package(urdfdom REQUIRED)
    endif()

    set(IDYNTREE_MODELIO_URDF_HEADERS include/iDynTree/ModelIO/URDFSensorsImport.h
                                      include/iDynTree/ModelIO/URDFGenericSensorsImport.h
                                      include/iDynTree/ModelIO/URDFModelImport.h
                                      include/iDynTree/ModelIO/URDFDofsImport.h)

    set(IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS
                                    include/iDynTree/ModelIO/impl/urdf_compatibility.h
                                    include/iDynTree/ModelIO/impl/urdf_export.hpp
                                    include/iDynTree/ModelIO/impl/urdf_import.hpp
                                    include/iDynTree/ModelIO/impl/urdf_sensor_import.hpp
                                    include/iDynTree/ModelIO/impl/urdf_generic_sensor_import.hpp
                                    include/iDynTree/ModelIO/impl/config.h)

    set(IDYNTREE_MODELIO_URDF_SOURCES src/urdf_export.cpp
                                    src/urdf_import.cpp
                                    src/urdf_sensor_import.cpp
                                    src/urdf_generic_sensor_import.cpp
                                    src/URDFSensorsImport.cpp
                                    src/URDFGenericSensorsImport.cpp
                                    src/URDFModelImport.cpp
                                    src/URDFDofsImport.cpp)

    source_group("Source Files" FILES ${IDYNTREE_MODELIO_URDF_SOURCES})
    source_group("Header Files" FILES ${IDYNTREE_MODELIO_URDF_HEADERS})
    source_group("Private Header Files" FILES ${IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS})

    include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS}
                            ${urdfdom_INCLUDE_DIRS})

    set(libraryname idyntree-modelio-urdf)

    add_library(${libraryname} ${IDYNTREE_MODELIO_URDF_SOURCES}
                               ${IDYNTREE_MODELIO_URDF_HEADERS}
                               ${IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS})


    target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                      ${orocos_kdl_INCLUDE_DIRS}
                                                      ${IDYNTREE_TREE_INCLUDE_DIRS}
                                              PRIVATE ${TinyXML_INCLUDE_DIRS})

    target_link_libraries(${libraryname} idyntree-core
                                         idyntree-model
                                         idyntree-sensors
                                         idyntree-kdl
                                         ${orocos_kdl_LIBRARIES}
                                        ${urdfdom_LIBRARIES}
                                        ${console_bridge_LIBRARIES}
                                        ${TinyXML_LIBRARIES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                 ${orocos_kdl_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_URDF_HEADERS})
set_property(TARGET ${libraryname} PROPERTY PRIVATE_HEADER ${IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS})

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO/impl)

    if(IDYNTREE_COMPILE_TESTS)
        add_subdirectory(tests)
    endif()
else()
    set(IDYNTREE_MODELIO_URDF_HEADERS include/iDynTree/ModelIO/URDFModelImport.h
                                      include/iDynTree/ModelIO/URDFDofsImport.h)

    set(IDYNTREE_MODELIO_URDF_SOURCES src/URDFModelImport.cpp
                                      src/URDFDofsImport.cpp)

    SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_URDF_SOURCES})
    SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_URDF_HEADERS})

    # share headers with all iDynTree targets
    set(libraryname idyntree-modelio-urdf)

    add_library(${libraryname} ${IDYNTREE_MODELIO_URDF_SOURCES} ${IDYNTREE_MODELIO_URDF_HEADERS})

    target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                              PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                      ${TinyXML_INCLUDE_DIRS})

    target_link_libraries(${libraryname} PUBLIC idyntree-model
                                         PRIVATE ${TinyXML_LIBRARIES})

    set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_URDF_HEADERS})

    install(TARGETS ${libraryname}
            EXPORT iDynTree
            COMPONENT runtime
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
            LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
            ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

    set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

    get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})
endif()