/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Estimation/SimpleLeggedOdometry.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/ModelTransformers.h>

#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

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
                                const Transform& world_H_initialFixedFrame)
{
    FrameIndex initialFixedFrameIndex = this->m_model.getFrameIndex(initialFixedFrame);

    return init(initialFixedFrameIndex,world_H_initialFixedFrame);
}


bool SimpleLeggedOdometry::init(const FrameIndex initialFixedFrameIndex,
                                const Transform& world_H_initialFixedFrame)
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
    Transform initialFixedFrame_H_fixedLink = m_model.getFrameTransform(initialFixedFrameIndex).inverse();
    m_world_H_fixedLink = world_H_initialFixedFrame*initialFixedFrame_H_fixedLink;

    m_isOdometryInitialized = true;

    return true;
}

bool SimpleLeggedOdometry::init(const std::string& initialFixedFrame,
                                const std::string& initialWorldFrame)
{
    return this->init(m_model.getFrameIndex(initialFixedFrame),
                      m_model.getFrameIndex(initialWorldFrame));
}


bool SimpleLeggedOdometry::init(const FrameIndex initialFixedFrameIndex,
                                const FrameIndex initialWorldFrameIndex)
{
    if( !m_isModelValid )
    {
         reportError("SimpleLeggedOdometry",
                     "init",
                     "Model not initialised.");
         return false;
    }

    if( !m_model.isValidFrameIndex(initialFixedFrameIndex) ||
        !m_model.isValidFrameIndex(initialWorldFrameIndex)
    )
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

    // Set the fixed link
    m_fixedLinkIndex = m_model.getFrameLink(initialFixedFrameIndex);

    // TODO : we need a simple class to get arbitrary transform in a model,
    //        something even simpler then KinDynComputations to address this
    //        kind of computation that appear from time to time 
    // Set the world_H_fixedLink transform
    LinkIndex linkAttachedToWorldFrameIndex = m_model.getFrameLink(initialWorldFrameIndex);
    Transform worldFrame_H_linkAttachedToWorldFrame = m_model.getFrameTransform(initialWorldFrameIndex).inverse();
    Transform linkAttachedToWorldFrameIndex_H_floatingBase = m_base_H_link(linkAttachedToWorldFrameIndex).inverse();
    Transform floatingBase_H_fixedLink = m_base_H_link(m_fixedLinkIndex);

    m_world_H_fixedLink = worldFrame_H_linkAttachedToWorldFrame*linkAttachedToWorldFrameIndex_H_floatingBase*floatingBase_H_fixedLink;

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


bool SimpleLeggedOdometry::loadModelFromFile(const std::string filename,
                                             const std::string /*filetype*/)
{
    Model _model;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDF(filename,_model);

    if( !parsingCorrect )
    {
        reportError("SimpleLeggedOdometry",
                    "loadModelFromFile",
                    "Error in parsing model from URDF.");
        return false;
    }

    return setModel(_model);
}

bool SimpleLeggedOdometry::loadModelFromFileWithSpecifiedDOFs(const std::string filename,
                                                              const std::vector< std::string >& consideredDOFs,
                                                              const std::string filetype)
{
    Model _modelFull;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDF(filename,_modelFull);

    if( !parsingCorrect )
    {
        reportError("SimpleLeggedOdometry",
                    "loadModelFromFileWithSpecifiedDOFs",
                    "Error in parsing model from URDF.");
        return false;
    }


    Model _modelReduced;

    // Create a reduced model: this will lump all not considered joints
    iDynTree::createReducedModel(_modelFull,consideredDOFs,_modelReduced);

    return setModel(_modelReduced);
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

    assert(m_fixedLinkIndex < m_base_H_link.getNrOfLinks());
    Transform base_H_fixed = m_base_H_link(m_fixedLinkIndex);
    Transform base_H_link =  m_base_H_link(link_index);

    return m_world_H_fixedLink*base_H_fixed.inverse()*base_H_link;
}


}

