/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Estimation/GravityCompensationHelpers.h"

namespace iDynTree 
{

  GravityCompensationHelper::GravityCompensationHelper() : m_isModelValid(false),
                                                           m_isKinematicsUpdated(false),
                                                           m_model(),
                                                           m_dynamicTraversal(),
                                                           m_kinematicTraversals(),
                                                           m_jointPos(),
                                                           m_linkVels(),
                                                           m_linkProperAccs(),
                                                           m_linkNetExternalWrenchesZero(),
                                                           m_linkIntWrenches(),
                                                           m_generalizedTorques()
  {

  }
  
  GravityCompensationHelper::~GravityCompensationHelper()
  {
    freeKinematicTraversals();
  }
  
  void GravityCompensationHelper::allocKinematicTraversals(const size_t nrOfLinks)
  {
    m_kinematicTraversals.resize(nrOfLinks);
    for (size_t link = 0; link < m_kinematicTraversals.size(); link++)
    {
      m_kinematicTraversals[link] = new iDynTree::Traversal;
    }
  }
  
  void GravityCompensationHelper::freeKinematicTraversals()
  {
    for (size_t link = 0; link < m_kinematicTraversals.size(); link++)
    {
      delete m_kinematicTraversals[link];
    }
    m_kinematicTraversals.resize(0);
  }
  
  bool GravityCompensationHelper::loadModel(const Model& _model, const std::string dynamicBase)
  {
    m_model = _model;
    
    // resize the data structures
    iDynTree::LinkIndex dynamicBaseIndex = m_model.getLinkIndex(dynamicBase);
    
    bool ok = m_model.computeFullTreeTraversal(m_dynamicTraversal, dynamicBaseIndex);
    if (!ok)
    {
      m_isModelValid = false;
      return false;
    }
    
    freeKinematicTraversals();
    allocKinematicTraversals(m_model.getNrOfLinks());
    
    m_jointPos.resize(m_model);
    m_jointDofsZero.resize(m_model);
    m_jointDofsZero.zero();
    m_linkVels.resize(m_model);
    m_linkProperAccs.resize(m_model);
    m_linkIntWrenches.resize(m_model);
    m_linkNetExternalWrenchesZero.resize(m_model);
    m_linkNetExternalWrenchesZero.zero();
    m_generalizedTorques.resize(m_model);

    // set the model valid
    m_isModelValid = true;
    return true;
  }
  
  bool GravityCompensationHelper::updateKinematicsFromGravity(const JointPosDoubleArray& jointPos, const FrameIndex& floatingFrame, const Vector3& gravity)
  {
    if (!m_isModelValid)
    {
      iDynTree::reportError("GravityCompensationHelper", "updateKinematicsFromGravity", "Model and sensors information is not set");
      return false;
    }
    
    iDynTree::Vector3 properClassicalAcceleration;
    properClassicalAcceleration(0) = -gravity(0);
    properClassicalAcceleration(1) = -gravity(1);
    properClassicalAcceleration(2) = -gravity(2);
    
    return updateKinematicsFromProperAcceleration(jointPos, floatingFrame, properClassicalAcceleration);
  }
  
  bool GravityCompensationHelper::updateKinematicsFromProperAcceleration(const JointPosDoubleArray& jointPos, const FrameIndex& floatingFrame, const Vector3& properClassicalLinearAcceleration)
  {
    if (!m_isModelValid)
    {
      iDynTree::reportError("GravityCompensationHelper", "updateKinematicsFromProperAcceleration", "Model and sensors information is not set");
      return false;
    }
    
    if (floatingFrame == FRAME_INVALID_INDEX || floatingFrame < 0 || floatingFrame >= (int)m_model.getNrOfFrames())
    {
      iDynTree::reportError("GravityCompensationHelper", "updateKinematicsFromProperAcceleration", "Unknown frame index specified" );
      return false;
    }
    
    // Get link index of the specified frame
    iDynTree::LinkIndex floatingLinkIndex = m_model.getFrameLink(floatingFrame);
    
    // Build the traversal if it is not present
    if (m_kinematicTraversals[floatingLinkIndex]->getNrOfVisitedLinks() == 0)
    {
      m_model.computeFullTreeTraversal(*m_kinematicTraversals[floatingLinkIndex], floatingLinkIndex);
    }
    
    // To initialize the kinmatic propagation, we should first convert the kinematics
    // information from the frame in which they are specified to the main frame of the link
    iDynTree::Transform link_H_frame = m_model.getFrameTransform(floatingFrame);
    
    iDynTree::Twist base_vel_frame, base_vel_link;
    iDynTree::SpatialAcc base_acc_frame, base_acc_link;
    iDynTree::ClassicalAcc base_classical_acc_link;
    
    iDynTree::Vector3 zero3;
    zero3.zero();
    
    base_vel_frame.setLinearVec3(zero3);
    base_vel_frame.setAngularVec3(zero3);
    base_vel_link = link_H_frame*base_vel_frame;
    
    base_acc_frame.setLinearVec3(properClassicalLinearAcceleration);
    base_acc_frame.setAngularVec3(zero3);
    base_acc_link = link_H_frame*base_acc_frame;
    
    base_classical_acc_link.fromSpatial(base_acc_link, base_vel_link);
    
    // Propagate the kinematics information
    bool ok = iDynTree::dynamicsEstimationForwardVelAccKinematics(m_model, *(m_kinematicTraversals[floatingLinkIndex]),
                                                                  base_classical_acc_link.getLinearVec3(),
                                                                  base_vel_link.getAngularVec3(),
                                                                  base_classical_acc_link.getAngularVec3(),
                                                                  jointPos, m_jointDofsZero, m_jointDofsZero,
                                                                  m_linkVels, m_linkProperAccs);
    // store joint positions
    m_jointPos = jointPos;
    
    if (!ok)
    {
      return false;
    }
    else
    {
      m_isKinematicsUpdated = true;
      return true;
    }
  }
  
  bool GravityCompensationHelper::getGravityCompensationTorques(JointDOFsDoubleArray& jointTrqs)
  {
    if (!m_isModelValid)
    {
      iDynTree::reportError("GravityCompensationHelper", "getGravityCompensationTorques", "Model not set");
      return false;
    }
    
    if (!m_isKinematicsUpdated)
    {
      iDynTree::reportError("GravityCompensationHelper", "getGravityCompensationTorques", "Kinematic information not set");
      return false;
    }
    
    // Compute joint torques
    bool ok = iDynTree::RNEADynamicPhase(m_model, m_dynamicTraversal, m_jointPos, m_linkVels, m_linkProperAccs,
                                         m_linkNetExternalWrenchesZero, m_linkIntWrenches, m_generalizedTorques);
    if (!ok)
    {
      iDynTree::reportError("GravityCompensationHelper", "getGravityCompensationTorques", "Error in computing RNEADynamicPhase");
      return false;
    }
    
    // store computed torques
    jointTrqs = m_generalizedTorques.jointTorques();
    
    return true;
  }

}

