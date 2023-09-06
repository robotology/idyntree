// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/EigenHelpers.h>

#include <iDynTree/SimpleLeggedOdometry.h>

#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/Indices.h>
#include <iDynTree/ModelTransformers.h>

#include <iDynTree/ModelLoader.h>

#include <sstream>

namespace iDynTree
{

SimpleLeggedOdometry::SimpleLeggedOdometry(): m_model(),
                                              m_traversal(),
                                              m_isModelValid(false),
                                              m_kinematicsUpdated(false),
                                              m_isOdometryInitialized(false),
                                              m_fixedLinkIndex(iDynTree::LINK_INVALID_INDEX),
                                              m_world_H_fixedLink(Transform::Identity())
{
}

SimpleLeggedOdometry::~SimpleLeggedOdometry()
{
}

bool SimpleLeggedOdometry::init(const std::string& initialFixedFrame,
                                const Transform initialFixedFrame_H_world)
{
    FrameIndex initialFixedFrameIndex = this->m_model.getFrameIndex(initialFixedFrame);

    return init(initialFixedFrameIndex,initialFixedFrame_H_world);
}


bool SimpleLeggedOdometry::init(const FrameIndex initialFixedFrameIndex,
                                const Transform initialFixedFrame_H_world)
{
    if( !m_isModelValid )
    {
         reportError("SimpleLeggedOdometry",
                     "init",
                     "Model not initialised.");
         return false;
    }

    if( !m_model.isValidFrameIndex(initialFixedFrameIndex) )
    {
        reportError("SimpleLeggedOdometry",
                    "init","invalid frame passed");
        return false;
    }

    m_fixedLinkIndex = m_model.getFrameLink(initialFixedFrameIndex);

    Transform world_H_initialFixedFrame = initialFixedFrame_H_world.inverse();
    Transform initalFixedFrame_H_fixedLink =  m_model.getFrameTransform(initialFixedFrameIndex).inverse();

    m_world_H_fixedLink = world_H_initialFixedFrame*initalFixedFrame_H_fixedLink;

    m_isOdometryInitialized = true;

    return true;
}


bool SimpleLeggedOdometry::init(const std::string & initialFixedFrame,
                                const std::string & initalReferenceFrameForWorld,
                                const Transform initialReferenceFrame_H_world)
{
    FrameIndex initialFixedFrameIndex = this->m_model.getFrameIndex(initialFixedFrame);
    FrameIndex initalReferenceFrameIndexForWorld = this->m_model.getFrameIndex(initalReferenceFrameForWorld);

    return init(initialFixedFrameIndex,initalReferenceFrameIndexForWorld,initialReferenceFrame_H_world);
}


bool SimpleLeggedOdometry::init(const FrameIndex initialFixedFrameIndex,
                                const FrameIndex initalReferenceFrameIndexForWorld,
                                const Transform initialReferenceFrame_H_world)
{
    if( !m_isModelValid )
    {
         reportError("SimpleLeggedOdometry",
                     "init",
                     "Model not initialised.");
         return false;
    }

    if( !m_model.isValidFrameIndex(initialFixedFrameIndex) ||
        !m_model.isValidFrameIndex(initalReferenceFrameIndexForWorld) )
    {
        reportError("SimpleLeggedOdometry",
                    "init","invalid frame passed");
        return false;
    }

    if( ! m_kinematicsUpdated )
    {
        reportError("SimpleLeggedOdometry",
                    "init","updateKinematics never called");
        return false;
    }

    m_fixedLinkIndex = m_model.getFrameLink(initialFixedFrameIndex);
    LinkIndex linkAttachedToWorldIndex = m_model.getFrameLink(initalReferenceFrameIndexForWorld);

    Transform world_H_initialReferenceFrame = initialReferenceFrame_H_world.inverse();
    Transform initalReferenceFrame_H_linkAttachedToWorld =  m_model.getFrameTransform(initalReferenceFrameIndexForWorld).inverse();
    Transform linkAttachedToWorld_H_floatingBase = m_base_H_link(linkAttachedToWorldIndex).inverse();
    Transform floatingBase_H_fixedLink           = m_base_H_link(m_fixedLinkIndex);

    m_world_H_fixedLink = world_H_initialReferenceFrame*initalReferenceFrame_H_linkAttachedToWorld*linkAttachedToWorld_H_floatingBase*floatingBase_H_fixedLink;

    m_isOdometryInitialized = true;

    return true;
}

bool SimpleLeggedOdometry::updateKinematics(JointPosDoubleArray& jointPos)
{
    if( !m_isModelValid )
    {
        reportError("SimpleLeggedOdometry",
                    "updateKinematics","model not valid");
        return false;
    }

    if( !jointPos.isConsistent(m_model) )
    {
        reportError("SimpleLeggedOdometry",
                    "updateKinematics","error in size of input jointPos");
        return false;
    }

    bool ok = ForwardPositionKinematics(m_model,m_traversal,
                                        Transform::Identity(),jointPos,
                                        m_base_H_link);

    m_kinematicsUpdated = ok;

    return ok;
}


const Model& SimpleLeggedOdometry::model() const
{
    return m_model;
}

bool SimpleLeggedOdometry::setModel(const Model& _model)
{
    m_model = _model;

    // resize the data structures
    m_model.computeFullTreeTraversal(m_traversal);

    // set that the model is valid
    m_isModelValid = true;

    // Set the kinematics and the odometry is not initialized
    m_isOdometryInitialized          = false;
    m_kinematicsUpdated              = false;

    // Resize the linkPositions
    m_base_H_link.resize(m_model);

    return true;
}

std::string SimpleLeggedOdometry::getCurrentFixedLink()
{
    if( this->m_isModelValid && this->m_isOdometryInitialized )
    {
        return this->m_model.getLinkName(this->m_fixedLinkIndex);
    }
    else
    {
        reportError("SimpleLeggedOdometry",
                    "getCurrentFixedLink",
                    "getCurrentFixedLink was called, but no model is available or the odometry is not initialized.");
        return "";
    }
}

bool SimpleLeggedOdometry::changeFixedFrame(const FrameIndex newFixedFrame)
{
    if( !this->m_kinematicsUpdated )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the kinematics info was never setted.");
        return false;
    }

    LinkIndex newFixedLink = this->m_model.getFrameLink(newFixedFrame);

    if( newFixedLink == LINK_INVALID_INDEX )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the provided new fixed frame is unknown.");
        return false;
    }

    Transform world_H_oldFixed = this->m_world_H_fixedLink;
    LinkIndex oldFixedLink = m_fixedLinkIndex;
    Transform oldFixed_H_newFixed = m_base_H_link(oldFixedLink).inverse()*m_base_H_link(newFixedLink);
    Transform world_H_newFixed = world_H_oldFixed*oldFixed_H_newFixed;
    this->m_world_H_fixedLink = world_H_newFixed;
    this->m_fixedLinkIndex = newFixedLink;

    return true;
}

bool SimpleLeggedOdometry::changeFixedFrame(const FrameIndex newFixedFrame, const Transform & world_H_newFixedFrame)
{
    if( !this->m_kinematicsUpdated )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the kinematics info was never setted.");
        return false;
    }

    LinkIndex newFixedLink = this->m_model.getFrameLink(newFixedFrame);

    if( newFixedLink == LINK_INVALID_INDEX )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the provided new fixed frame is unknown.");
        return false;
    }

    Transform newFixedFrame_H_newFixedLink = m_model.getFrameTransform(newFixedFrame).inverse();
    this->m_world_H_fixedLink = world_H_newFixedFrame * newFixedFrame_H_newFixedLink;
    this->m_fixedLinkIndex = newFixedLink;

    return true;
}

bool SimpleLeggedOdometry::changeFixedFrame(const std::string& newFixedFrame, const Transform & world_H_newFixedFrame)
{
    iDynTree::FrameIndex newFixedFrameIndex = this->m_model.getFrameIndex(newFixedFrame);

    if( newFixedFrameIndex == FRAME_INVALID_INDEX )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the provided new fixed frame is unknown.");
        return false;
    }

    return this->changeFixedFrame(newFixedFrameIndex, world_H_newFixedFrame);
}

bool SimpleLeggedOdometry::changeFixedFrame(const std::string& newFixedFrame)
{
    iDynTree::FrameIndex newFixedFrameIndex = this->m_model.getFrameIndex(newFixedFrame);

    if( newFixedFrameIndex == FRAME_INVALID_INDEX )
    {
        reportError("SimpleLeggedOdometry",
                    "changeFixedFrame",
                    "changeFixedFrame was called, but the provided new fixed frame is unknown.");
        return false;
    }

    return this->changeFixedFrame(newFixedFrameIndex);
}

Transform SimpleLeggedOdometry::getWorldLinkTransform(const LinkIndex link_index)
{
    if( !this->m_kinematicsUpdated || !this->m_isOdometryInitialized  )
    {
        reportError("SimpleLeggedOdometry",
                    "getWorldLinkTransform",
                    "getWorldLinkTransform was called, but the kinematics update or the odometry init was never setted.");
        return Transform::Identity();
    }

    if( !this->m_model.isValidLinkIndex(link_index) )
    {
        reportError("SimpleLeggedOdometry",
                    "getWorldLinkTransform",
                    "getWorldLinkTransform was called, but the request linkindex is not part of the model");
        return Transform::Identity();
    }

    assert(m_fixedLinkIndex < static_cast<LinkIndex>(m_base_H_link.getNrOfLinks()));
    Transform base_H_fixed = m_base_H_link(m_fixedLinkIndex);
    Transform base_H_link =  m_base_H_link(link_index);

    return m_world_H_fixedLink*base_H_fixed.inverse()*base_H_link;
}

Transform SimpleLeggedOdometry::getWorldFrameTransform(const FrameIndex frame_index)
{
    if( !this->m_kinematicsUpdated || !this->m_isOdometryInitialized  )
    {
        reportError("SimpleLeggedOdometry",
                    "getWorldFrameTransform",
                    "getWorldLinkTransform was called, but the kinematics update or the odometry init was never setted.");
        return Transform::Identity();
    }

    if( !this->m_model.isValidFrameIndex(frame_index) )
    {
        reportError("SimpleLeggedOdometry",
                    "getWorldFrameTransform",
                    "getWorldLinkTransform was called, but the request linkindex is not part of the model");
        return Transform::Identity();
    }

    LinkIndex linkIndex = this->m_model.getFrameLink(frame_index);
    Transform link_H_frame = m_model.getFrameTransform(frame_index);
    return getWorldLinkTransform(linkIndex) * link_H_frame;
}

}
