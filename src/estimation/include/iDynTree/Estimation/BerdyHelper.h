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

#ifndef IDYNTREE_BERDY_HELPER_H
#define IDYNTREE_BERDY_HELPER_H

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

/**
 * Enumeration describing the possible sensor types implemented in Berdy.
 *
 * Note that the concept of "sensor" in Berdy estimation is more general that
 * just a physical sensor mounted on the robot: for example it can include
 * the information that a link is fixed to the ground (i.e. its angular velocity,
 * angular and linear acceleration are zero) even if this information is not coming
 * from an actual physical sensors. For this reason we do not use directly
 * the iDynTree::SensorTypes enum, even if we reserve the first 1000 elements o
 * of this enum to be compatibile with the iDynTree::SensorTypes enum.
 * Enum values duplicates between BerdySensorTypes and SensorTypes are append a
 * _SENSOR suffix to avoid problems when wrapping such enum wit SWIG.
 */
enum BerdySensorTypes
{
    SIX_AXIS_FORCE_TORQUE_SENSOR = SIX_AXIS_FORCE_TORQUE,
    ACCELEROMETER_SENSOR = ACCELEROMETER,
    GYROSCOPE_SENSOR = GYROSCOPE,
    DOF_ACCELERATION_SENSOR = 1000,
    DOF_TORQUE_SENSOR       = 1001,
    NET_EXT_WRENCH_SENSOR   = 1002,
    /**
     * Non-physical sensor that measures the wrench trasmitted by a joint.
     */
    JOINT_WRENCH_SENSOR     = 1003
};

bool isLinkBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);
bool isJointBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);
bool isDOFBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType);

/**
 * Options of the BerdyHelper class.
 *
 * Documentation of each option is provided as usual Doxygen documentation.
 * Default values for each options are specified in the contstructor.
 */
struct BerdyOptions
{
public:
    BerdyOptions() : berdyVariant(ORIGINAL_BERDY_FIXED_BASE),
                     includeAllNetExternalWrenchesAsDynamicVariables(true),
                     includeAllJointAccelerationsAsSensors(true),
                     includeAllJointTorquesAsSensors(false),
                     includeAllNetExternalWrenchesAsSensors(true),
                     includeFixedBaseExternalWrench(false)
    {
    }

    /**
     * Type of berdy variant implemented.
     *
     * For description of each type of variant
     * check the BerdyVariants enum documentation.
     *
     * Default value: ORIGINAL_BERDY_FIXED_BASE .
     */
    BerdyVariants berdyVariant;

    /**
     * If true, include the net external wrenches in the dynamic variables vector.
     *
     * Default value: true .
     *
     * \note the Net effect of setting this to zero is to consider all the external
     *       wrenches to be equal to 0. If berdyVariant is ORIGINAL_BERDY_FIXED_BASE,
     *       the "net" external wrenches does not include the wrench at the base.
     */
    bool includeAllNetExternalWrenchesAsDynamicVariables;

    /**
     * If true, include the joint accelerations in the sensors vector.
     *
     * Default value: true .
     */
    bool includeAllJointAccelerationsAsSensors;

    /**
     * If true, include the joint torques in the sensors vector.
     *
     * Default value: false .
     */
    bool includeAllJointTorquesAsSensors;

    /**
     * If true, include the net external wrenches in the sensors vector.
     * It is not compatible with the use of includeAllNetExternalWrenchesAsDynamicVariables set to false.
     *
     * Default value: true .
     */
    bool includeAllNetExternalWrenchesAsSensors;

    /**
     * If includeNetExternalWrenchesAsSensors is true and the
     * variant is ORIGINAL_BERDY_FIXED_BASE, if this is
     * true the external wrench acting on the base fixed link
     * is included in the sensors.
     *
     * Default value : false .
     */
    bool includeFixedBaseExternalWrench;

    /**
     * Vector of joint names for which we assume that a virtual
     * measurement of the wrench trasmitted on the joint is available.
     *
     * \note This measurements are tipically used only for debug, actual
     *       internal wrenches are tipically measured using a SIX_AXIS_FORCE_TORQUE_SENSOR .
     */
    std::vector<std::string> jointOnWhichTheInternalWrenchIsMeasured;

    /**
     * Check that the options are not self-contradicting.
     */
    bool checkConsistency();
};


//Unfortunately some sensors used in berdy are not proper sensors.
//I cannot use the Sensor class which has almost all the information needed
/**
 * Structure which describes the essential information about a sensor used in berdy
 * A sensor is identified by the pair (type, id)
 */
struct BerdySensor {
    iDynTree::BerdySensorTypes type; /*<! type of the sensor */
    std::string id; /*<! ID of the sensor */
    iDynTree::IndexRange range; /*<! Range of the sensor
                                 * (starting location in the measurements equations
                                 *  and number of measuremes equations associated with the sensor */

    bool operator==(const struct BerdySensor&);
};

    struct BerdyDynamicVariable {
        iDynTree::BerdyDynamicVariablesTypes type;
        std::string id;
        iDynTree::IndexRange range;
    };

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
     * Options of the current berdy variant used.
     */
    BerdyOptions m_options;

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

    std::vector<BerdySensor> m_sensorsOrdering; /*<! Sensor ordering. Created on init */
    std::vector<BerdyDynamicVariable> m_dynamicVariablesOrdering; /*<! Dynamic variable ordering. Created on init */

    /**
     * Helpers method for initialization.
     */
    bool initOriginalBerdyFixedBase();
    bool initBerdyFloatingBase();
    bool initSensorsMeasurements();

    /**
     * Ranges of dynamic variables
     */
    IndexRange getRangeOriginalBerdyFixedBase(const BerdyDynamicVariablesTypes dynamicVariableType, const TraversalIndex idx);
    IndexRange getRangeLinkVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const LinkIndex idx);
    IndexRange getRangeJointVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const JointIndex idx);
    IndexRange getRangeDOFVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const DOFIndex idx);

    /**
     * Range
     */
    IndexRange getRangeSensorVariable(const SensorType type, const unsigned int sensorIdx);
    IndexRange getRangeDOFSensorVariable(const BerdySensorTypes sensorType, const DOFIndex idx);
    IndexRange getRangeJointSensorVariable(const BerdySensorTypes sensorType, const JointIndex idx);
    IndexRange getRangeLinkSensorVariable(const BerdySensorTypes sensorType, const LinkIndex idx);

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

    void cacheSensorsOrdering();
    void cacheDynamicVariablesOrdering();

    /**
     * Helper for mapping sensors measurements to the Y vector.
     */
    struct {
        size_t dofAccelerationOffset;
        size_t dofTorquesOffset;
        size_t netExtWrenchOffset;
        size_t jointWrenchOffset;
    } berdySensorTypeOffsets;

    /**
     * Helper of additional sensors.
     */
    struct {
      /**
       * List of joint wrench sensors.
       */
      std::vector<JointIndex> wrenchSensors;
      /**
       * Mapping between jndIx in wrenchSensor and
       */
      std::vector<size_t> jntIdxToOffset;
    } berdySensorsInfo;

    /**
     * Buffer for sensor serialization.
     */
    VectorDynSize realSensorMeas;

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
     *
     * @param[in] model The used model.
     * @param[in] sensors The used sensors.
     * @param[in] options The used options, check BerdyOptions docs.
     * @return true if all went well, false otherwise.
     */
    bool init(const Model& model,
              const SensorsList& sensors,
              const BerdyOptions options=BerdyOptions());

    /**
     * Get currently used options.
     */
    BerdyOptions getOptions() const;

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
     * Resize and set to zero Berdy matrices.
     */
    bool resizeAndZeroBerdyMatrices(MatrixDynSize & D, VectorDynSize & bD,
                                    MatrixDynSize & Y, VectorDynSize & bY);

    /**
     * Get Berdy matrices
     */
    bool getBerdyMatrices(MatrixDynSize & D, VectorDynSize & bD,
                          MatrixDynSize & Y, VectorDynSize & bY);


    /**
     * Return the internal ordering of the sensors
     *
     * Measurements are expected to respect the internal sensors ordering
     * Use this function to obtain the sensors ordering.
     *
     * @return the sensors ordering
     */
    const std::vector<BerdySensor>& getSensorsOrdering() const;

    const std::vector<BerdyDynamicVariable>& getDynamicVariablesOrdering() const;

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
     * Serialize sensor variable from separate buffers.
     */
    bool serializeSensorVariables(SensorsMeasurements     & sensMeas,
                                  LinkNetExternalWrenches & netExtWrenches,
                                  JointDOFsDoubleArray    & jointTorques,
                                  JointDOFsDoubleArray    & jointAccs,
                                  LinkInternalWrenches    & linkJointWrenches,
                                  VectorDynSize& y);

    /**
     * Debug function:
     *
     * \note This method has been added for debug, and could be removed soon.
     */
    bool serializeDynamicVariablesComputedFromFixedBaseRNEA(JointDOFsDoubleArray  & jointAccs,
                                                            LinkNetExternalWrenches & netExtWrenches,
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

    /**
     * Set the kinematic information necessary for the dynamics estimation assuming that a
     * given  baseframe (specified by the m_dynamicsTraversal) is not accelerating with respect to the inertial frame.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] jointVel the velocities of the joints of the model.
     * @param[in] jointAcc the accelerations of the joints of the model.
     * @param[in] gravity the gravity acceleration vector, expressed in the specified fixed frame.
     *
     * \note gravity is used only if selected BerdyVariant is ORIGINAL_BERDY_FIXED_BASE.
     *
     * @return true if all went ok, false otherwise.
     *
     * @note this is equivalent to updateKinematicsFromFixedBase(jointPos,jointVel,m_dynamicsTraversal.getBaseLink()->getIndex(),gravity);
     *
     */
    bool updateKinematicsFromTraversalFixedBase(const JointPosDoubleArray  & jointPos,
                                                const JointDOFsDoubleArray & jointVel,
                                                const Vector3 & gravity);

    //@}


    /**
      * @name Methods to get informations on the serialization used.
      */
    //@{

    /**
     * Get a human readable description of the elements of the dynamic variables.
     */
    //std::string getDescriptionOfDynamicVariables();

    //@}



};


}

#endif
