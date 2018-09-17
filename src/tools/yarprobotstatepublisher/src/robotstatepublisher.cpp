/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file coordinator.cpp
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <yarp/math/Math.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/yarp/YARPConversions.h>

#include "robotstatepublisher.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/************************************************************/
JointStateSuscriber::JointStateSuscriber(): m_module(nullptr)
{
}

/************************************************************/
void JointStateSuscriber::attach(YARPRobotStatePublisherModule* module)
{
    m_module = module;
}

/************************************************************/
void JointStateSuscriber::onRead(yarp::rosmsg::sensor_msgs::JointState& v)
{
    m_module->onRead(v);
}

/************************************************************/
YARPRobotStatePublisherModule::YARPRobotStatePublisherModule(): m_iframetrans(nullptr),
                                                                m_usingNetworkClock(false),
                                                                m_baseFrameName(""),
                                                                m_baseFrameIndex(iDynTree::FRAME_INVALID_INDEX),
                                                                m_buf4x4(4,4)
{
}


/************************************************************/
bool YARPRobotStatePublisherModule::configure(ResourceFinder &rf)
{
    string name="yarprobotstatepublisher";
    m_rosNode = new yarp::os::Node("/yarprobotstatepublisher");
    string modelFileName=rf.check("model",Value("model.urdf")).asString();
    m_period=rf.check("period",Value(0.010)).asDouble();

    Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    pTransformclient_cfg.put("local", "/"+name+"/transformClient");
    pTransformclient_cfg.put("remote", "/transformServer");

    m_tfPrefix = rf.check("tfPrefix",Value("")).asString();

    bool ok_client = m_ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        yError()<<"Is the transform server running?";
        close();
        return false;
    }
    if (!m_ddtransformclient.view(m_iframetrans))
    {
        yError()<<"IFrameTransform I/F is not implemented";
        close();
        return false;
    }

    // If YARP is using a network clock, writing on a ROS topic is not working
    // Workaround: explicitly instantiate a network clock to read the time from gazebo
    if( yarp::os::NetworkBase::exists("/clock") )
    {
        m_usingNetworkClock = true;
        m_netClock.open("/clock");
    }

    // Open the model
    string pathToModel=rf.findFileByName(modelFileName);
    bool ok = m_kinDynComp.loadRobotModelFromFile(pathToModel);
    if (!ok || !m_kinDynComp.isValid())
    {
        yError()<<"Impossible to load file " << pathToModel;
        close();
        return false;
    }

    // Resize the joint pos buffer
    m_jointPos.resize(m_kinDynComp.model().getNrOfPosCoords());

    // Get the base frame information
    if (rf.check("base-frame"))
    {
        m_baseFrameName = rf.find("base-frame").asString();
    }
    else
    {
        // If base-frame is not passed, use the default base-frame of the model
        const iDynTree::Model& model = m_kinDynComp.model();
        m_baseFrameName = model.getLinkName(model.getDefaultBaseLink());
    }

    const iDynTree::Model& model = m_kinDynComp.model();
    m_baseFrameIndex = model.getFrameIndex(m_baseFrameName);

    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError()<<"Impossible to find frame " << m_baseFrameName << " in the model";
        close();
        return false;
    }

    // Setup the topic and configureisValid the onRead callback
    string jointStatesTopicName = rf.check("jointstates-topic",Value("/joint_states")).asString();
    m_jointStateSubscriber = new JointStateSuscriber();
    m_jointStateSubscriber->attach(this);
    m_jointStateSubscriber->topic(jointStatesTopicName);
    m_jointStateSubscriber->useCallback();

    return true;
}


/************************************************************/
bool YARPRobotStatePublisherModule::close()
{
    yarp::os::LockGuard guard(m_mutex);

    // Disconnect the topic subscriber
    if (m_jointStateSubscriber)
    {
        m_jointStateSubscriber->interrupt();
        m_jointStateSubscriber->close();
        delete m_jointStateSubscriber;
    }

    if (m_ddtransformclient.isValid())
    {
        yInfo()<<"Closing the tf device";
        m_ddtransformclient.close();
        m_iframetrans = nullptr;
    }

    m_baseFrameIndex = iDynTree::FRAME_INVALID_INDEX;

    if(m_rosNode)
        delete m_rosNode;
    m_rosNode = nullptr;

    return true;
}


/************************************************************/
double YARPRobotStatePublisherModule::getPeriod()
{
    return m_period;
}



/************************************************************/
bool YARPRobotStatePublisherModule::updateModule()
{
    // All the actual processing is performed in the onRead callback
    return true;
}

/************************************************************/
void YARPRobotStatePublisherModule::onRead(yarp::rosmsg::sensor_msgs::JointState &v)
{
    yarp::os::LockGuard guard(m_mutex);

    // If configure was successful, parse the data
    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        return;
    }

    // Check size
    if (v.name.size() != m_jointPos.size())
    {
        yError() << "Size mismatch. Model has " << m_jointPos.size()
                 << " joints, while the received JointState message has " << v.name.size() << " joints.";
        return;
    }

    // TODO: this part can be drastically speed up.
    //      Possible improvements:
    //        * Add a map string --> indeces
    // Fill the buffer of joints positions
    const iDynTree::Model& model = m_kinDynComp.model();
    for (size_t i=0; i < v.name.size(); i++)
    {
        iDynTree::JointIndex jntIndex = model.getJointIndex(v.name[i]);
        if (jntIndex == iDynTree::JOINT_INVALID_INDEX)
            continue;

        m_jointPos(model.getJoint(jntIndex)->getDOFsOffset()) = v.position[i];
    }

    // Set the updated joint positions
    m_kinDynComp.setJointPos(m_jointPos);

    // Publish the frames on TF
    bool m_publishGlobalTF = false; // TODO
    if (m_publishGlobalTF) {
        // Read the last TF from the world (ground) to the base
        yarp::sig::Matrix world_H_base_yarp;
        if (!m_iframetrans->getTransform(m_tfPrefix + m_baseFrameName, "ground", world_H_base_yarp)) {
            yError() << "Failed to retrieve the relative transform between ground and"
                     << m_tfPrefix + m_baseFrameName;
            return;
        }

        // Convert the transform from Yarp to iDynTree
        iDynTree::Transform world_H_base;
        iDynTree::toiDynTree(world_H_base_yarp, world_H_base);

        for (size_t frameIdx=0; frameIdx < model.getNrOfFrames(); frameIdx++) {
            // skip self-tranform
            if (m_baseFrameIndex == frameIdx) {
                continue;
            }

            // skip publishing the world to base transform
            if (model.getFrameName(frameIdx) == m_baseFrameName) {
                continue;
            }

            iDynTree::Transform base_H_frame = m_kinDynComp.getRelativeTransform(m_baseFrameIndex, frameIdx);
            iDynTree::toYarp(world_H_base * base_H_frame.asHomogeneousTransform(), m_buf4x4);
            m_iframetrans->setTransform(m_tfPrefix + model.getFrameName(frameIdx),
                                        "ground",
                                        m_buf4x4);
        }
    }
    else {
        for (size_t frameIdx=0; frameIdx < model.getNrOfFrames(); frameIdx++)
        {
            if(m_baseFrameIndex == frameIdx)    // skip self-tranform
                continue;

            iDynTree::Transform base_H_frame = m_kinDynComp.getRelativeTransform(m_baseFrameIndex, frameIdx);
            iDynTree::toYarp(base_H_frame.asHomogeneousTransform(), m_buf4x4);
            m_iframetrans->setTransform(m_tfPrefix + model.getFrameName(frameIdx),
                                        m_tfPrefix + model.getFrameName(m_baseFrameIndex),
                                        m_buf4x4);
        }
    }

    return;
}
