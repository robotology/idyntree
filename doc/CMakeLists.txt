# Copyright: (C) 2009 RobotCub Consortium
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Doxygen)

if(DOXYGEN_FOUND)
    set(DOX_GENERATE_HTML YES)
    set(DOX_GENERATE_XML NO)
    option(IDYNTREE_DOXYGEN_XML "Generate doxygen XML output, for use by SWIG" NO)
    mark_as_advanced(IDYNTREE_DOXYGEN_XML)
    if(IDYNTREE_DOXYGEN_XML)
        set(DOX_GENERATE_XML YES)
    endif()
    set(IDYNTREE_DOXYGEN_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH
        "Install directory for iDynTree doxygen documentation")
    mark_as_advanced(IDYNTREE_DOXYGEN_INSTALL_DIR)
    # iDynTree documentation is always verbose now.  It used to be
    # partially stripped of stuff less interesting to end-users.
    set(DOX_EXTRA_DIR src conf doc/spec)
    set(DOX_PATTERNS "*.h *.dox *.cpp *.hpp")
    set(DOX_GENERATE_MAN NO)

    # search mscgen program
    # for setting the MSCGEN_PATH variable in the Doxyfile
    find_program(MSCGEN_EXECUTABLE mscgen)

    if( MSCGEN_EXECUTABLE STREQUAL "MSCGEN_EXECUTABLE-NOTFOUND" )
        message(STATUS "mscgen program not found")
        set(MSCGEN_PATH "")
    else()
        # set the MSCGEN_PATH variable by stripping the name (mscgen)
        # from the MSCGEN_EXECUTABLE
        string(REGEX REPLACE "mscgen$" "" MSCGEN_PATH ${MSCGEN_EXECUTABLE})
    endif()

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    add_custom_target(dox COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set(DOX_GENERATE_HTML NO)
    set(DOX_GENERATE_MAN YES)
    set(DOX_GENERATE_XML NO)
    set(DOX_PATTERNS "cmd_*.dox")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.man)
    add_custom_target(man COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.man)
endif()
