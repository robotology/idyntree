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
