# Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

set(QtiDynTreeSoleGuiPlugin_SRCS
                             qticubskinguiplugin.cpp
                             qticubskinguiplugin_plugin.cpp
                             ObserverThread.cpp)

set(QtiDynTreeSoleGuiPlugin_HDRS qticubskinguiplugin.h
                             qticubskinguiplugin_plugin.h
                             include/ObserverThread.h)

source_group("Source Files" FILES ${QtiDynTreeSoleGuiPlugin_SRCS})
source_group("Header Files" FILES ${QtiDynTreeSoleGuiPlugin_HDRS})

set(QtiDynTreeSoleGuiPlugin_QMLDIR "robotology/idyntree/solegui")

add_library(QtiDynTreeSoleGuiPlugin MODULE ${QtiDynTreeSoleGuiPlugin_SRCS}
                                       ${QtiDynTreeSoleGuiPlugin_HDRS})
target_link_libraries(QtiDynTreeSoleGuiPlugin YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_sig
                                              YARP::YARP_dev
                                              ${GSL_LIBRARIES}
                                              idyntree-high-level
                                              idyntree-inverse-kinematics
                                              ctrlLib)
target_link_libraries(QtiDynTreeSoleGuiPlugin Qt5::Quick)

qticub_qml_plugin(QtiDynTreeSoleGuiPlugin "${QtiDynTreeSoleGuiPlugin_QMLDIR}")

install(TARGETS QtiDynTreeSoleGuiPlugin
        COMPONENT utilities
        RUNTIME DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtiDynTreeSoleGuiPlugin_QMLDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtiDynTreeSoleGuiPlugin_QMLDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtiDynTreeSoleGuiPlugin_QMLDIR}")

install(FILES qmldir DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtiDynTreeSoleGuiPlugin_QMLDIR}")
