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

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/yarp/YARPConversions.h>

#include "robotstatepublisher.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/************************************************************/
JointStateSubscriber::JointStateSubscriber(): m_module(nullptr)
{
}

/************************************************************/
void JointStateSubscriber::attach(YARPRobotStatePublisherModule* module)
{
    m_module = module;
}

/************************************************************/
void JointStateSubscriber::onRead(yarp::rosmsg::sensor_msgs::JointState& v)
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
    string namePrefix = rf.check("name-prefix",Value("")).asString();
    string robot = rf.check("robot",Value("")).asString();
    if (!namePrefix.empty()) {
        m_rosNode.reset(new yarp::os::Node("/"+namePrefix+"/yarprobotstatepublisher"));
    }
    else if (!robot.empty()) {
        m_rosNode.reset(new yarp::os::Node("/"+robot+"/yarprobotstatepublisher"));
        std::cerr << "[WARNING] The yarprobotstatepublisher option robot is deprecated," << std::endl <<
                     "[WARNING] use name-prefix option instead";
    }
    else {
        m_rosNode.reset(new yarp::os::Node("/yarprobotstatepublisher"));
    }

    string modelFileName=rf.check("model",Value("model.urdf")).asString();
    m_period=rf.check("period",Value(0.010)).asDouble();
    m_treeType=rf.check("tree-type", Value("SHALLOW")).asString();
    if(m_treeType != "SHALLOW" && m_treeType != "DEEP")
    {
        yError("Wrong tree format. The only allowed values are \"SHALLOW\" or \"DEEP\"");
        return false;
    }

    Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    if (!namePrefix.empty()) {
        pTransformclient_cfg.put("local", "/"+namePrefix+"/"+name+"/transformClient");
    }
    else pTransformclient_cfg.put("local", "/"+name+"/transformClient");

    pTransformclient_cfg.put("remote", "/transformServer");

    m_tfPrefix = rf.check("tf-prefix",Value("")).asString();

    bool ok_client = m_ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        yError()<<"Is the transformServer YARP device running?";
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
    iDynTree::ModelLoader modelLoader;
    bool ok = modelLoader.loadModelFromFile(pathToModel);
    ok = ok && m_kinDynComp.loadRobotModel(modelLoader.model());
    if (!ok || !m_kinDynComp.isValid())
    {
        yError()<<"Impossible to load file " << pathToModel;
        close();
        return false;
    }

    // Resize the joint pos buffer
    m_jointPos.resize(m_kinDynComp.model().getNrOfPosCoords());

    // Initilize the joint pos buffer to Zero
    m_jointPos.zero();

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

    // Set reduced model option
    // By default TFs of all the frames in the model are streamed
    // If the option is present, only the TFs of the links are streamed to transform server
    this->reducedModelOption=rf.check("reduced-model");

    // Setup the topic and configureisValid the onRead callback
    string jointStatesTopicName = rf.check("jointstates-topic",Value("/joint_states")).asString();
    m_jointStateSubscriber.reset(new JointStateSubscriber());
    m_jointStateSubscriber->attach(this);
    m_jointStateSubscriber->topic(jointStatesTopicName);
    m_jointStateSubscriber->useCallback();

    return true;
}


/************************************************************/
bool YARPRobotStatePublisherModule::close()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // Disconnect the topic subscriber
    if (m_jointStateSubscriber)
    {
        m_jointStateSubscriber->interrupt();
        m_jointStateSubscriber->close();
    }

    if (m_ddtransformclient.isValid())
    {
        yInfo()<<"Closing the tf device";
        m_ddtransformclient.close();
        m_iframetrans = nullptr;
    }

    m_baseFrameIndex = iDynTree::FRAME_INVALID_INDEX;

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
    std::lock_guard<std::mutex> guard(m_mutex);

    // If configure was successful, parse the data
    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        return;
    }

    // TODO: this part can be drastically speed up.
    //      Possible improvements:
    //        * Add a map string --> indeces
    // Fill the buffer of joints positions
    const iDynTree::Model& model = m_kinDynComp.model();
    iDynTree::JointIndex jntIndex;
    for (size_t i=0; i < v.name.size(); i++)
    {
        jntIndex = model.getJointIndex(v.name[i]);

        if ( jntIndex == iDynTree::JOINT_INVALID_INDEX)
            continue;
        if (!(model.getJoint(jntIndex)->getNrOfDOFs()))
            continue;

        m_jointPos(model.getJoint(jntIndex)->getDOFsOffset()) = v.position[i];
    }

    // Set the updated joint positions
    m_kinDynComp.setJointPos(m_jointPos);

    // Set the size of the tf frames to be published
    size_t sizeOfTFFrames;
    if (this->reducedModelOption)
    {
        sizeOfTFFrames = model.getNrOfLinks();
    }
    else
    {
        sizeOfTFFrames = model.getNrOfFrames();
    }

    if (m_treeType == "SHALLOW")
    {
        // In shallow mode, we publish the position of each frame of the robot w.r.t. to the base frame of the robot
        for (size_t frameIdx=0; frameIdx < sizeOfTFFrames; frameIdx++)
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
    else
    {
        // mode == DEEP
        // In deep mode, we need to distinguish the following cases:
        // For the frames that are frames of the link, we publish their location w.r.t. to their parent link
        // For the additional frames, we publish their location w.r.t. to the frame of the link to which they are
        // attached (note that this transform are actually constant)

        // The traversal is the data structure that contains information on which link is parent of which other link,
        // as in iDynTree the model is an undirected data structure
        iDynTree::Traversal traversal;

        // We generate a traversal using the base frame index
        m_kinDynComp.model().computeFullTreeTraversal(traversal, m_baseFrameIndex);

        bool setOk = false;

        //Processing joints instead of links since it's easier this way to distinguish between static transform and non static ones
        for (size_t jointIndex=0; jointIndex < model.getNrOfJoints(); jointIndex++)
        {
            auto currJoint = model.getJoint(jointIndex);
            iDynTree::LinkIndex parentLinkIndex = traversal.getParentLinkIndexFromJointIndex(model,jointIndex);//currJoint->getFirstAttachedLink();
            iDynTree::LinkIndex linkIndex = traversal.getChildLinkIndexFromJointIndex(model,jointIndex);//currJoint->getSecondAttachedLink();
            iDynTree::Transform parentLink_H_link = m_kinDynComp.getRelativeTransform(parentLinkIndex, linkIndex);
            iDynTree::toYarp(parentLink_H_link.asHomogeneousTransform(), m_buf4x4);

            if(currJoint->getNrOfDOFs() == 0) //Static transform
            {
                //To avoid setting a static transform more than once
                if(m_iframetrans->canTransform(m_tfPrefix + model.getFrameName(linkIndex),m_tfPrefix + model.getFrameName(parentLinkIndex)))
                {
                    continue;
                }
                setOk = m_iframetrans->setTransformStatic(m_tfPrefix + model.getFrameName(linkIndex),
                                                          m_tfPrefix + model.getFrameName(parentLinkIndex),
                                                          m_buf4x4);
            }
            else
            {
                setOk = m_iframetrans->setTransform(m_tfPrefix + model.getFrameName(linkIndex),
                                                    m_tfPrefix + model.getFrameName(parentLinkIndex),
                                                    m_buf4x4);
            }

            if(!setOk)
            {
                yInfo("The transformation between %s and %s cannot be set as %s",(m_tfPrefix + model.getFrameName(parentLinkIndex)).c_str(),
                      (m_tfPrefix + model.getFrameName(linkIndex)).c_str(),currJoint->getNrOfDOFs()==0?"static":"timed");
            }
        }

        // Process frames, only if the reduced model option is not passed
        if (!this->reducedModelOption)
        {
            // Process additional frames (that have all indexes between model.getNrOfLinks()+1 and model.getNrOfFrames()
            for (size_t frameIndex=model.getNrOfLinks(); frameIndex < model.getNrOfFrames(); frameIndex++)
            {
                iDynTree::LinkIndex linkIndex = m_kinDynComp.model().getFrameLink(frameIndex);
                iDynTree::Transform link_H_frame = m_kinDynComp.model().getFrameTransform(frameIndex);
                iDynTree::toYarp(link_H_frame.asHomogeneousTransform(), m_buf4x4);

                //To avoid setting a static transform more than once
                if(m_iframetrans->canTransform(m_tfPrefix + model.getFrameName(frameIndex),m_tfPrefix + model.getFrameName(linkIndex)))
                {
                    continue;
                }
                setOk = m_iframetrans->setTransformStatic(m_tfPrefix + model.getFrameName(frameIndex),
                                                          m_tfPrefix + model.getFrameName(linkIndex),
                                                          m_buf4x4);
                if(!setOk)
                    yInfo("The transformation between %s and %s cannot be set",(m_tfPrefix + model.getFrameName(linkIndex)).c_str(),
                          (m_tfPrefix + model.getFrameName(frameIndex)).c_str());
            }
        }
    }

    return;
}
