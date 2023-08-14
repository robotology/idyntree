# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(Doxygen)

if(DOXYGEN_FOUND)
    set(DOX_GENERATE_HTML YES)
    set(DOX_GENERATE_XML NO)

    # Options related to doxygen
    option(IDYNTREE_DOXYGEN_XML "Generate doxygen XML output, for use by SWIG" NO)
    mark_as_advanced(IDYNTREE_DOXYGEN_XML)

    option(IDYNTREE_DOXYGEN_VERBOSE "Generate a verbose HTML output for Doxygen, including call and caller graphs." OFF)
    mark_as_advanced(IDYNTREE_DOXYGEN_VERBOSE)
    set(DOX_VERBOSE NO)
    if(IDYNTREE_DOXYGEN_VERBOSE)
        set(DOX_VERBOSE YES)
    endif()


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

    # search dot program
    # for setting the DOT_PATH variable in the Doxyfile
    find_program(DOT_EXECUTABLE dot)

    if( DOT_EXECUTABLE STREQUAL "DOT_EXECUTABLE-NOTFOUND" )
        message(STATUS "dot program not found")
        set(DOT_PATH "")
    else()
        # set the DOT_PATH variable by stripping the name (dot)
        # from the DOT_EXECUTABLE
        string(REGEX REPLACE "dot$" "" DOT_PATH ${DOT_EXECUTABLE})
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
