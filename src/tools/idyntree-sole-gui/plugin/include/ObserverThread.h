// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef IDYNTREE_SOLE_GUI_OBSERVER_THREAD_H
#define IDYNTREE_SOLE_GUI_OBSERVER_THREAD_H

#include <string>
#include <vector>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

#include <iDynTree/Core/Direction.h>
#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <iCub/ctrl/minJerkCtrl.h>

using namespace yarp::dev;

class QPainter;

class ObserverThread : public yarp::os::RateThread
{
protected:
    // Attribute to read encoders position
    yarp::dev::PolyDriver m_wholeBodyDevice;
    yarp::dev::IEncoders * m_iencs;
    yarp::dev::IPositionDirect * m_iposdir;
    iDynTree::KinDynComputations m_kinDyn;
    yarp::sig::Vector m_measuredEncodersInDeg;
    yarp::sig::Vector m_desiredJointPositionsInDeg;
    std::string m_primarySole;
    std::string m_secondarySole;

    // Attribute to read pressure sensors
    bool m_skip_pressure;
    yarp::dev::PolyDriver m_primarySolePressure;
    yarp::dev::IAnalogSensor* m_primaryPressureAnalog;
    yarp::dev::PolyDriver m_secondarySolePressure;
    yarp::dev::IAnalogSensor* m_secondaryPressureAnalog;
    std::vector<iDynTree::Position> m_primaryPressurePadsInPrimarySole;
    std::vector<iDynTree::Position> m_secondaryPressurePadsInSecondarySole;
    yarp::sig::Vector m_measuredPressurePrimarySoleInN;
    yarp::sig::Vector m_measuredPressureSecondarySoleInN;

    yarp::os::BufferedPort<yarp::os::Bottle> skin_port;

    // Attributes to read gravity direction
    bool m_skip_gravity;
    yarp::os::BufferedPort<yarp::sig::Vector> m_imuPort;
    std::string m_imuFrameName;
    std::string m_remoteImuPortName;
    std::string m_localImuPortName;
    iDynTree::Direction m_gravityDirection;
    iCub::ctrl::FirstOrderLowPassFilter *filtIMUGravity;    //!< filter the gravity vector received from the IMU

    int sensorsNum;
    bool mbSimpleDraw;

    // Shared data (populated by the run, and used by the draw)
    std::mutex m_mutex;
    iDynTree::Position m_comInPrimarySole;
    iDynTree::Position m_desiredComInPrimarySole;

    iDynTree::Position m_primaryCopInPrimarySole;
    iDynTree::Position m_secondaryCopInPrimarySole;
    iDynTree::Position m_totalCopInPrimarySole;

    iDynTree::Transform m_primarySole_X_secondarySole;



public:
    ObserverThread(yarp::os::ResourceFinder& config,int period);
    bool init(yarp::os::ResourceFinder& config);
    ~ObserverThread();

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    void draw(QPainter* qpainter, int widthInPixels, int heightInPixels);
    void drawPolygon(QPainter* qpainter, iDynTree::Polygon& polygon);
};

#endif
