# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(expression-parser CXX)

include_directories(include/expression_parser)


set(EXPR_PARSER_SOURCES  src/error.cpp
                         src/functions.cpp
                         src/parser.cpp
                         src/variablelist.cpp)

set(EXPR_PARSER_HEADERS include/expression_parser/constants.h
                        include/expression_parser/error.h
                        include/expression_parser/functions.h
                        include/expression_parser/parser.h
                        include/expression_parser/variablelist.h)

SOURCE_GROUP("ExpressionParser\\Source Files" FILES ${SYMORO_PAR_SRCS})
SOURCE_GROUP("ExpressionParser\\Header Files" FILES ${SYMORO_PAR_HPPS})

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(LIB_TYPE STATIC)
elseif(BUILD_SHARED_LIBS)
    set(LIB_TYPE SHARED)
else()
    set(LIB_TYPE STATIC)
endif()


add_library(${PROJECT_NAME} ${LIB_TYPE} ${EXPR_PARSER_HEADERS} ${EXPR_PARSER_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
                      COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
                      PUBLIC_HEADER "${EXPR_PARSER_HEADERS}"
                  )

install(TARGETS ${PROJECT_NAME}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})
