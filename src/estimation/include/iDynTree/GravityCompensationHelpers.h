// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef GRAVITY_COMPENSATION_HELPERS_H
#define GRAVITY_COMPENSATION_HELPERS_H

#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/Dynamics.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Utils.h>
#include <iDynTree/ClassicalAcc.h>

#include <iDynTree/ExternalWrenchesEstimation.h>

namespace iDynTree
{
  /** 
   * @brief Class computing the gravity compensation torques 
   * using accelerometer measurements
   * 
   * @note the estimation of the gravity assumes 
   * the non-gravitational accelerations measured by the
   * accelerometer as negligible
   * 
   */
  class GravityCompensationHelper
  {
  public:
    /**
     * @brief Default Constructor
     */
    GravityCompensationHelper();
    
    /**
     * @brief Destructor
     */
    ~GravityCompensationHelper();
    
    /**
     * @brief Load model
     * @return true if successful, false otherwise
     */
    bool loadModel(const iDynTree::Model& _model, const std::string dynamicBase);
    
    /**
     * @brief Set the kinematic information necessary for the gravity torques estimation using 
     * the proper acceleration coming from an accelerometer
     * 
     * @note the estimation of the gravity assumes the non-gravitational accelerations 
     * measured by the accelerometer as negligible. 
     * 
     * @param[in] jointPos the position of the joints in the model
     * @param[in] floatingFrame the frame index for which proper acceleration is provided
     * @param[in] properClassicalLinearAcceleration proper (actual acceleration - gravity)
     *                                              classical acceleration of the origin of the
     *                                              specified frame, expresssed in the specified 
     *                                              frame orientation
     *
     * @return true if successful, false otherwise   
     */
    bool updateKinematicsFromProperAcceleration(const iDynTree::JointPosDoubleArray& jointPos,
                                                const iDynTree::FrameIndex& floatingFrame,
                                                const iDynTree::Vector3& properClassicalLinearAcceleration);
    
    /**
     * @brief Set the kinematic information necessary for the gravity torques estimation using 
     * the assumed known gravity vector on frame
     * 
     * @note This is implemented as updateKinematicsFromProperAcceleration(jointPos, floatingFrame, -gravity);
     * 
     * @param[in] jointPos the position of the joints in the model
     * @param[in] floatingFrame the frame index for which gravity vector is provided
     * @param[in] properClassicalLinearAcceleration gravity acceleration of the origin of the
     *                                              specified frame, expresssed in the specified 
     *                                              frame orientation
     *
     * @return true if successful, false otherwise   
     */
    bool updateKinematicsFromGravity(const iDynTree::JointPosDoubleArray& jointPos,
                                     const iDynTree::FrameIndex& floatingFrame,
                                     const iDynTree::Vector3& gravity);
    
    /**
     * @brief Get the gravity compensation torques
     * @return true if successful, false otherwise
     */
    bool getGravityCompensationTorques(iDynTree::JointDOFsDoubleArray& jointTrqs);
    
  private:
    /**
     * @brief Helper function for kinematic traversal dynamic allocation
     * @param[in] nrOfLinks number of links to be allocated in the traversal
     */
    void allocKinematicTraversals(const size_t nrOfLinks);
    
    /**
     * @brief Helper function for kinematic traversal dynamic deallocation
     */
    void freeKinematicTraversals();
    
    bool m_isModelValid;          ///< flag to check validity of the model
    bool m_isKinematicsUpdated;   ///< flag to check if kinematics of the robot is updated
    iDynTree::Model m_model;      ///< robot model for gravity compensation estimation
    
    iDynTree::Traversal m_dynamicTraversal; ///< Traversal used for dynamic computations
    
    /**
     * Vector of Traversal used for the kinematic computations.
     * m_kinematicTraversals[l] contains the traversal with base link l .
     */
    std::vector<iDynTree::Traversal*> m_kinematicTraversals;
    
    iDynTree::JointPosDoubleArray m_jointPos;                         ///< joint positions
    iDynTree::JointDOFsDoubleArray m_jointDofsZero;                   ///< zero DOFArrays for joint velocities and joint accelerations
    iDynTree::LinkVelArray m_linkVels;                                ///< link velocities
    iDynTree::LinkAccArray m_linkProperAccs;                          ///< link proper accelerations
    iDynTree::LinkNetExternalWrenches m_linkNetExternalWrenchesZero;  ///< link external wrenches set to zero
    iDynTree::LinkInternalWrenches m_linkIntWrenches;                 ///< link internal wrenches
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedTorques;    ///< generalized torques 
  };
}
#endif

