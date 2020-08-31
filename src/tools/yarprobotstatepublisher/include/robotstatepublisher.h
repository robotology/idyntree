/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file robotstatepublisher.h
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#ifndef YARP_ROBOT_STATE_PUBLISHER_H
#define YARP_ROBOT_STATE_PUBLISHER_H

#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/rosmsg/sensor_msgs/JointState.h>

#include <mutex>
#include <memory>

class YARPRobotStatePublisherModule;

/****************************************************************/
class JointStateSubscriber: public yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::JointState>
{
private:
    YARPRobotStatePublisherModule* m_module;

public:
    JointStateSubscriber();
    void attach(YARPRobotStatePublisherModule* module);
    using yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::JointState>::onRead;
    virtual void        onRead(yarp::rosmsg::sensor_msgs::JointState &v);
};


/****************************************************************/
class YARPRobotStatePublisherModule : public yarp::os::RFModule
{
    double m_period;
    yarp::dev::PolyDriver       m_ddtransformclient;
    yarp::dev::IFrameTransform       *m_iframetrans;

    std::string m_tfPrefix;
    std::string m_treeType;

    // Clock-related workaround
    bool m_usingNetworkClock;
    yarp::os::NetworkClock m_netClock;

    // Reduced flag option
    bool reducedModelOption;

    // Class for computing forward kinematics
   iDynTree::KinDynComputations m_kinDynComp;
   iDynTree::VectorDynSize m_jointPos;
   std::string m_baseFrameName;
   iDynTree::FrameIndex m_baseFrameIndex;
   yarp::sig::Matrix m_buf4x4;

   // Mutex protecting the method across the different threads
   std::mutex m_mutex;

   // /JointState topic scruscriber
   std::unique_ptr<yarp::os::Node> m_rosNode;
   std::unique_ptr<JointStateSubscriber> m_jointStateSubscriber;

public:
    YARPRobotStatePublisherModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
    virtual void        onRead(yarp::rosmsg::sensor_msgs::JointState &v);
};

#endif

