# Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


# Remove -rdynamic from CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS.
# This is needed for some weird reason, or executables will not load resources
# from the plugins.
string(REPLACE "-rdynamic" "" CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS}")


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qtquick2applicationviewer)

set(iDynTreeSoleGui_SRCS main.cpp)
set(iDynTreeSoleGui_HDRS )
set(iDynTreeSoleGui_QRC_FILES qml.qrc)
set(iDynTreeSoleGui_QML_FILES main.qml)

qt5_add_resources(iDynTreeSoleGui_QRC_GEN_SRCS ${iDynTreeSoleGui_QRC_FILES})

source_group("Source Files" FILES ${iDynTreeSoleGui_SRCS})
source_group("Header Files" FILES ${iDynTreeSoleGui_HDRS})
source_group("Resources Files" FILES ${iDynTreeSoleGui_QRC_FILES})
source_group("QML Files" FILES ${iDynTreeSoleGui_QML_FILES})
source_group("Generated Files" FILES ${iDynTreeSoleGui_QRC_GEN_SRCS})

qticub_use_qml_plugin()

add_executable(idyntree-sole-gui WIN32 ${iDynTreeSoleGui_SRCS}
                                       ${iDynTreeSoleGui_HDRS}
                                       ${iDynTreeSoleGui_QRC_GEN_SRCS})

target_link_libraries(idyntree-sole-gui Qt5::Widgets Qt5::Qml Qt5::Quick)


install(TARGETS idyntree-sole-gui
        COMPONENT utilities
        DESTINATION ${CMAKE_INSTALL_BINDIR})

