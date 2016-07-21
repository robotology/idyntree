/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IDYNTREE_BERDY_HELPERS_H
#define IDYNTREE_BERDY_HELPERS_H

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <vector>

namespace iDynTree
{

/**
 * Enumeration of the Berdy variants implemented
 * in this class.
 */
enum BerdyVariants
{
    /**
     * Original version of Berdy, as described in:
     *
     * Latella, C.; Kuppuswamy, N.; Romano, F.; Traversaro, S.; Nori, F.
     * Whole-Body Human Inverse Dynamics with Distributed Micro-Accelerometers, Gyros and Force Sensing. Sensors 2016, 16, 727.
     * http://www.mdpi.com/1424-8220/16/5/727
     *
     * The original version of Berdy is assuming that the joint numbering is a regular ordering of links and joints.
     * For this reason the serialization of link/joints quantities follows the one defined in the traversal.
     *
     * Furthremore, this version assumes that all joints have 1 Degree of freedom, so it does not work for models
     * with fixed joints.
     */
    ORIGINAL_BERDY_FIXED_BASE = 0,

    /**
     * Modified version of Berdy
     * for accounting for free floating dynamics and removing the NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV from the dynamic variables.
     *
     */
    BERDY_FLOATING_BASE = 1,
};


/**
 * Enumeration descriing the dynamic variables types (link acceleration, net wrenches, joint wrenches, joint torques, joint acceleration)
 * used in
 */
enum BerdyDynamicVariablesTypes
{
    //< a_i
    LINK_BODY_PROPER_ACCELERATION,
    // f^B_i
    NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,
    // f_i
    JOINT_WRENCH,
    // tau_i
    DOF_TORQUE,
    // f^x_i
    NET_EXT_WRENCH,
    // \ddot{q}_i
    DOF_ACCELERATION
};

bool isLinkBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);
bool isJointBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);
bool isDOFBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);

/**
 * \brief Helper class for computing Berdy matrices.
 *
 * Berdy refers to a class for algorithms to compute
 * the Maximum A Posteriori (MAP) estimation of the dynamic variables
 * of a multibody model (accelerations, external forces, joint torques)
 * assuming the knowledge the measurements of an arbitrary set of sensors and
 * of the kinematics and inertial characteristics of the model.
 *
 * The MAP estimation is computed using a sparse matrix representation of the multibody
 * dynamics Newton-Euler equations, originally described in:
 *
 * Latella, C.; Kuppuswamy, N.; Romano, F.; Traversaro, S.; Nori, F.
 * Whole-Body Human Inverse Dynamics with Distributed Micro-Accelerometers, Gyros and Force Sensing.
 * Sensors 2016, 16, 727.
 * http://www.mdpi.com/1424-8220/16/5/727
 *
 * Nori F, Kuppuswamy N, Traversaro S.
 * Simultaneous state and dynamics estimation in articulated structures.
 * In Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on 2015 Sep 28 (pp. 3380-3386). IEEE.
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7353848
 *
 *
 * This helper class implements two different variants of the Berdy dynamics representation, identified
 * with the BerdyVariants enum:
 * * The ORIGINAL_BERDY_FIXED_BASE is the original BERDY representation described in the papers,
 *   that was assuming a fixed base model with all the joints with 1-DOF .
 * * The BERDY_FLOATING_BASE is a variant in which the model is assumed to be floating,
 *   does not need the position or linear velocity of the floating base and supports
 *   joints with an arbitrary number of DOF .
 *
 * The sensors supported by this class are the one contained in the SensorsList representation,
 * which can be loaded directly from an URDF representation, see https://github.com/robotology/idyntree/blob/master/doc/model_loading.md#sensor-extensions .
 * Using this representation the following sensors can be loaded:
 *   * Internal Six-Axis Force Torque sensors
 *   * Gyroscopes
 *   * Accelerometers
 * Support for joint torques/acceleration or external wrenches measurements still needs to be added.
 *
 * \note The dynamics representation of Berdy is highly dependent on the link assumed to be the floating
 *       base of the robot, even in the case that BERDY_FLOATING_BASE is used. The assumed traversal (i.e.
 *       which link is the floating base and how the link are visited) is accessible with the dynamicTraversal() method.
 *
 * \note This class is still under heavy development, and its interface can change
 *       without any warning.
 */
class BerdyHelper
{
    /**
     * Model used in this class.
     */
    Model m_model;

    /**
     * Sensors used in this class
     */
    SensorsList m_sensors;

    /**
     * Traversal used for the dynamics computations
     */
    Traversal m_dynamicsTraversal;

    /**
     * Caches of traversals used for kinematic computations.
     */
    LinkTraversalsCache m_kinematicTraversals;

    /**
     * False initially, true after a valid model and sensors have been loaded.
     */
    bool m_areModelAndSensorsValid;

    /**
     * False initially, true after updateKinematics was successfully called.
     */
    bool m_kinematicsUpdated;

    /**
     * Type of berdy variant implemented,
     * for description of each type of variant
     * check the BerdyVariants enum documentation.
     */
    BerdyVariants m_berdyVariant;

    size_t m_nrOfDynamicalVariables;
    size_t m_nrOfDynamicEquations;
    size_t m_nrOfSensorsMeasurements;

    /**
     * Buffer of link-specific body velocities.
     */

    /**
     * Joint positions
     */
    JointPosDoubleArray m_jointPos;

    /**
     * Joint velocities
     */
    JointDOFsDoubleArray m_jointVel;


    /**
     * Link body velocities (i.e. 6D velocity of the link, expressed
     * in the link frame orientation and with respect to the link origin).
     */
    LinkVelArray m_linkVels;

    /**
     * Gravity expressed in the base link frame, used only
     * if getBerdyVariant is equal to ORIGINAL_BERDY_FIXED_BASE.
     */
    Vector3 m_gravity;
    SpatialAcc m_gravity6D;

    /**
     * Helpers method
     */
    bool initOriginalBerdyFixedBase();
    bool initBerdyFloatingBase();

    IndexRange getRangeOriginalBerdyFixedBase(const BerdyDynamicVariablesTypes dynamicVariableType, const TraversalIndex idx);
    IndexRange getRangeLinkVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const LinkIndex idx);
    IndexRange getRangeJointVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const JointIndex idx);
    IndexRange getRangeDOFVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const DOFIndex idx);
    IndexRange getRangeSensorVariable(const SensorType type, const unsigned int sensorIdx);

    /**
     * Ranges for specific dynamics equations
     */
    IndexRange getRangeLinkProperAccDynEq(const LinkIndex idx);

    /**
     * This used only by ORIGINAL_BERDY_FIXED_BASE, for BERDY_FLOATING_BASE this is
     * included in the JointWrench equations.
     */
    IndexRange getRangeLinkNetTotalwrenchDynEq(const LinkIndex idx);
    IndexRange getRangeJointWrench(const JointIndex idx);
    IndexRange getRangeDOFTorqueDynEq(const DOFIndex idx);


    bool computeBerdySensorMatrices(MatrixDynSize& Y, VectorDynSize& bY);
    bool computeBerdyDynamicsMatrices(MatrixDynSize& D, VectorDynSize& bD);
public:
    /**
     * Constructor
     */
    BerdyHelper();

    /**
     * Access the model.
     */
    Model& model();

    /**
     * Access the sensors.
     */
    SensorsList& sensors();

    /**
     * Acces the traveral used for the dynamics computations (const version)
     */
    const Traversal& dynamicTraversal();

    /**
     * Access the model (const version).
     */
    const Model& model() const;

    /**
     * Access the model (const version).
     */
    const SensorsList& sensors() const;



    /**
     * Init the class
     */
    bool init(const Model& model,
              const SensorsList& sensors,
              const BerdyVariants variant);

    /**
     * Get the used Berdy variant.
     */
    BerdyVariants getBerdyVariant() const;

    /**
     * Get the number of columns of the D matrix.
     * This depends on the Berdy variant selected.
     */
    size_t getNrOfDynamicVariables() const;

    /**
     * Get the number of dynamics equations used in the Berdy
     * equations
     */
    size_t getNrOfDynamicEquations() const;

    /**
     * Get the number of sensors measurements.
     */
    size_t getNrOfSensorsMeasurements() const;

    /**
     * Resize Berdy matrices
     */
    bool resizeBerdyMatrices(MatrixDynSize & D, VectorDynSize & bD,
                             MatrixDynSize & Y, VectorDynSize & bY);

    /**
     * Get Berdy matrices
     */
    bool getBerdyMatrices(MatrixDynSize & D, VectorDynSize & bD,
                          MatrixDynSize & Y, VectorDynSize & bY);

    /**
     * Serialized dynamic variables from the separate buffers
     */
    bool serializeDynamicVariables(LinkProperAccArray & properAccs,
                                   LinkNetTotalWrenchesWithoutGravity & netTotalWrenchesWithoutGrav,
                                   LinkNetExternalWrenches & netExtWrenches,
                                   LinkInternalWrenches    & linkJointWrenches,
                                   JointDOFsDoubleArray    & jointTorques,
                                   JointDOFsDoubleArray    & jointAccs,
                                   VectorDynSize& d);

     /**
      * @name Methods to submit the input data for dynamics computations.
      */
    //@{

    /**
     * Set the kinematic information necessary for the dynamics estimation using the
     * angular velocity information of a floating frame.
     *
     * \note This method cannot be used if the selected BerdyVariant is ORIGINAL_BERDY_FIXED_BASE.
     * \note we not require to give the linear velocity of floating base because the dynamics equations
     *       are invariant with respect to an offset in linear velocity. This convenient to avoid
     *       any dependency on any prior floating base estimation.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] jointVel the velocities of the joints of the model.
     * @param[in] jointAcc the accelerations of the joints of the model.
     * @param[in] floatingFrame the index of the frame for which kinematic information is provided.
     * @param[in] angularVel angular velocity (wrt to any inertial frame) of the specified floating frame,
     *                       expressed in the specified floating frame orientation.
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromFloatingBase(const JointPosDoubleArray  & jointPos,
                                          const JointDOFsDoubleArray & jointVel,
                                          const FrameIndex & floatingFrame,
                                          const Vector3 & angularVel);

    /**
     * Set the kinematic information necessary for the dynamics estimation assuming that a
     * given frame is not accelerating with respect to the inertial frame.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] jointVel the velocities of the joints of the model.
     * @param[in] jointAcc the accelerations of the joints of the model.
     * @param[in] fixedFrame the index of the frame that is not accelerating with respect to the inertial frame.
     * @param[in] gravity the gravity acceleration vector, expressed in the specified fixed frame.
     *
     * \note gravity is used only if selected BerdyVariant is ORIGINAL_BERDY_FIXED_BASE.
     *
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromFixedBase(const JointPosDoubleArray  & jointPos,
                                       const JointDOFsDoubleArray & jointVel,
                                       const FrameIndex & fixedFrame,
                                       const Vector3 & gravity);

    //@}

};


}

#endif