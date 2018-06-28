# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

# Specify a base directory where tests file can be located by the unit tests
set(IDYNTREE_MODELIO_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests/format_examples)

add_subdirectory(xml)
add_subdirectory(urdf)

if(IDYNTREE_USES_KDL)
    add_subdirectory(urdf-kdl)
endif()

option(IDYNTREE_ENABLE_SYMORO_PAR "Enable support for SyMoRo par format" TRUE)
if(IDYNTREE_ENABLE_SYMORO_PAR AND IDYNTREE_USES_KDL)
    add_subdirectory(symoro)
endif()

if(IDYNTREE_USES_ICUB_MAIN AND IDYNTREE_USES_KDL)
    add_subdirectory(iKin-kdl)
endif()

if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_KDL)
    add_subdirectory(tests)
endif()
