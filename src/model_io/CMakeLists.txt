# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


# Specify a base directory where tests file can be located by the unit tests
set(IDYNTREE_MODELIO_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests/format_examples)

add_subdirectory(xml)
add_subdirectory(codecs)
