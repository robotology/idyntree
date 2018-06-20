# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

set(IDYNTREE_MODELIO_XML_DATAFILES ${IDYNTREE_MODELIO_TESTFILES_DIR}/xml)

add_executable(XMLParserUnitTest XMLParserUnitTest.cpp)
target_compile_definitions(XMLParserUnitTest PUBLIC -DIDYNTREE_TEST_FILES_DIR="${IDYNTREE_MODELIO_XML_DATAFILES}")

target_include_directories(XMLParserUnitTest PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
target_link_libraries(XMLParserUnitTest idyntree-core idyntree-modelio-xml)
add_test(NAME UnitTestXMLParser COMMAND XMLParserUnitTest)

if(IDYNTREE_RUN_VALGRIND_TESTS)
    add_test(NAME memcheck_UnitTestXMLParser COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:XMLParserUnitTest>)
endif()