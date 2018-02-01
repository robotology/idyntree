/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_BERDY_HELPER_H
#define IDYNTREE_BERDY_HELPER_H

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/Triplets.h>

#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Model/LinkTraversalsCache.h>

#include <vector>

namespace iDynTree
{

/**
 * Enumeration of the Berdy variants implemented
 * in this class.
 *
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
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
    BERDY_FLOATING_BASE = 1
};

/**
 * Enumeration describing the dynamic variables types (link acceleration, net wrenches, joint wrenches, joint torques, joint acceleration)
 * used in Berdy
 *
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
 */
enum BerdyDynamicVariablesTypes
{
    /*!< \f$ a_i  \f$
     * Note that is is the **spatial** proper acceleration expressed,
     * i.e. the time derivative of the left-trivialized velocity minus the gravity expressed in body frame.
     **/
    LINK_BODY_PROPER_ACCELERATION,
    /*!< \f$ f^B_i \f$ */
    NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,
    /*!< \f$ f_i \f$ */
    JOINT_WRENCH,
    /*!< \f$ \tau_i \f$ */
    DOF_TORQUE,
    /*!< \f$ f^x_i \f$ */
    NET_EXT_WRENCH,
    /*!< \f$ \ddot{q}_i \f$ */
    DOF_ACCELERATION,
    /*!<
     * This is the classical proper acceleration,
     * i.e. the time derivative of the mixed velocity of the body frame minus the gravity expressed in body frame.
     * In Traversaro's PhD thesis this is called sensor proper acceleration.
     * This is the necessary for avoiding dependencies on the linear velocity of the base in the floating variant of
     * Berdy.
     */
    LINK_BODY_PROPER_CLASSICAL_ACCELERATION
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
 *
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
 */
enum BerdySensorTypes
{
    SIX_AXIS_FORCE_TORQUE_SENSOR = SIX_AXIS_FORCE_TORQUE,
    ACCELEROMETER_SENSOR = ACCELEROMETER,
    GYROSCOPE_SENSOR = GYROSCOPE,
    THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR = THREE_AXIS_ANGULAR_ACCELEROMETER,
    THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR  = THREE_AXIS_FORCE_TORQUE_CONTACT,
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
 *
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
 */
struct BerdyOptions
{
public:
    BerdyOptions() : berdyVariant(ORIGINAL_BERDY_FIXED_BASE),
                     includeAllNetExternalWrenchesAsDynamicVariables(true),
                     includeAllJointAccelerationsAsSensors(true),
                     includeAllJointTorquesAsSensors(false),
                     includeAllNetExternalWrenchesAsSensors(true),
                     includeFixedBaseExternalWrench(false),
                     baseLink("")
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
     *
     * \note if berdyVariant is BERDY_FLOATING_BASE, this option cannot be set to false.
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
     * Name of the link which will be considered as a base link for Berdy computations.
     *
     * \note If the string is empty the default base link of the model will be used.
     */
    std::string baseLink;

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
 *
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
 */
struct BerdySensor {
    iDynTree::BerdySensorTypes type; /*!< type of the sensor */
    std::string id; /*!< ID of the sensor */
    iDynTree::IndexRange range; /*!< Range of the sensor
                                 * (starting location in the measurements equations
                                 *  and number of measurements equations associated with the sensor) */

    /**
     * Overload of equality operator
     *
     * Two sensors are considered equals if they have the same type and id
     * @param sensor the sensor to which the current sensor is compared to
     * @return true if the two sensors are equal. False otherwise
     */
    bool operator==(const struct BerdySensor& sensor) const;

    bool operator<(const struct BerdySensor& sensor) const;
};

struct BerdyDynamicVariable {
    iDynTree::BerdyDynamicVariablesTypes type; /*!< type of the dynamic variable */
    std::string id; /*!< ID of the dynamic variable */
    iDynTree::IndexRange range; /*!< Range of the dynamic variable
                                 * (starting location in the dynamic equations
                                 *  and number of equations associated with the variable) */

    /**
     * Overload of equality operator
     *
     * Two variables are considered equals if they have the same type and id
     * @param variable the variable to which the current variable is compared to
     * @return true if the two variables are equal. False otherwise
     */
    bool operator==(const struct BerdyDynamicVariable& variable) const;

    bool operator<(const struct BerdyDynamicVariable& variable) const;
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
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
 * \ingroup iDynTreeExperimental
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

    std::vector<BerdySensor> m_sensorsOrdering; /*!< Sensor ordering. Created on init */
    std::vector<BerdyDynamicVariable> m_dynamicVariablesOrdering; /*!< Dynamic variable ordering. Created on init */

    /**
     * Helpers method for initialization.
     */
    bool initOriginalBerdyFixedBase();
    bool initBerdyFloatingBase();
    bool initSensorsMeasurements();

    IndexRange getRangeOriginalBerdyFixedBase(const BerdyDynamicVariablesTypes dynamicVariableType, const TraversalIndex idx) const;


    /**
     * Ranges for specific dynamics equations
     */

    // Dynamic equations for ORIGINAL_BERDY_FIXED_BASE
    IndexRange getRangeLinkProperAccDynEqFixedBase(const LinkIndex idx) const;
    IndexRange getRangeLinkNetTotalWrenchDynEqFixedBase(const LinkIndex idx) const;
    IndexRange getRangeJointWrenchDynEqFixedBase(const JointIndex idx) const;
    IndexRange getRangeDOFTorqueDynEqFixedBase(const DOFIndex idx) const;

    // Dynamic equations for BERDY_FLOATING_BASE
    IndexRange getRangeAccelerationPropagationFloatingBase(const LinkIndex idx) const;
    IndexRange getRangeNewtonEulerEquationsFloatingBase(const LinkIndex idx) const;

    bool computeBerdySensorMatrices(SparseMatrix<iDynTree::ColumnMajor>& Y, VectorDynSize& bY);
    bool computeBerdyDynamicsMatricesFixedBase(SparseMatrix<iDynTree::ColumnMajor>& D, VectorDynSize& bD);
    bool computeBerdyDynamicsMatricesFloatingBase(SparseMatrix<iDynTree::ColumnMajor>& D, VectorDynSize& bD);

    // Helper method
    Matrix6x1 getBiasTermJointAccelerationPropagation(IJointConstPtr joint,
                                                      const LinkIndex parentLinkIdx,
                                                      const LinkIndex childLinkIdx,
                                                      const Transform &child_X_parent);
    void cacheSensorsOrdering();
    void cacheDynamicVariablesOrderingFixedBase();
    void cacheDynamicVariablesOrderingFloatingBase();
    bool serializeDynamicVariablesFixedBase(LinkProperAccArray & properAccs,
                                            LinkNetTotalWrenchesWithoutGravity & netTotalWrenchesWithoutGrav,
                                            LinkNetExternalWrenches & netExtWrenches,
                                            LinkInternalWrenches    & linkJointWrenches,
                                            JointDOFsDoubleArray    & jointTorques,
                                            JointDOFsDoubleArray    & jointAccs,
                                            VectorDynSize& d);
    bool serializeDynamicVariablesFloatingBase(LinkProperAccArray & properAccs,
                                               LinkNetTotalWrenchesWithoutGravity & netTotalWrenchesWithoutGrav,
                                               LinkNetExternalWrenches & netExtWrenches,
                                               LinkInternalWrenches    & linkJointWrenches,
                                               JointDOFsDoubleArray    & jointTorques,
                                               JointDOFsDoubleArray    & jointAccs,
                                               VectorDynSize& d);

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

    Triplets matrixDElements;
    Triplets matrixYElements;

    /**
     * Transform between the frame in which the external net wrench measurements are expressed
     * and the link frames.
     */
    std::vector<Transform> m_link_H_externalWrenchMeasurementFrame;


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
    const Traversal& dynamicTraversal() const;

    /**
     * Access the model (const version).
     */
    const Model& model() const;

    /**
     * Access the model (const version).
     */
    const SensorsList& sensors() const;

    /**
     * Returns if the helper is valid.
     * The helper is valid if the model and the sensors have been loaded
     * @return true if the helper is valid. False otherwise
     */
    bool isValid() const;

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
    bool resizeAndZeroBerdyMatrices(SparseMatrix<iDynTree::ColumnMajor>& D, VectorDynSize &bD,
                                    SparseMatrix<iDynTree::ColumnMajor>& Y, VectorDynSize &bY);

    /**
     * Resize and set to zero Berdy matrices.
     *
     */
    bool resizeAndZeroBerdyMatrices(MatrixDynSize & D, VectorDynSize & bD,
                                    MatrixDynSize & Y, VectorDynSize & bY);

    /**
     * Get Berdy matrices
     */
    bool getBerdyMatrices(SparseMatrix<iDynTree::ColumnMajor>& D, VectorDynSize &bD,
                          SparseMatrix<iDynTree::ColumnMajor>& Y, VectorDynSize &bY);
    /**
     * Get Berdy matrices
     *
     * \note internally this function uses sparse matrices
     * Prefer the use of resizeAndZeroBerdyMatrices(SparseMatrix &, VectorDynSize &, SparseMatrix &, VectorDynSize &)
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

    /**
     * Get the range of the specified sensor in
     */
    IndexRange getRangeSensorVariable(const SensorType type, const unsigned int sensorIdx) const;
    IndexRange getRangeDOFSensorVariable(const BerdySensorTypes sensorType, const DOFIndex idx) const;
    IndexRange getRangeJointSensorVariable(const BerdySensorTypes sensorType, const JointIndex idx) const;
    IndexRange getRangeLinkSensorVariable(const BerdySensorTypes sensorType, const LinkIndex idx) const;

    /**
     * Ranges of dynamic variables
     */
    IndexRange getRangeLinkVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const LinkIndex idx) const;
    IndexRange getRangeJointVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const JointIndex idx) const;
    IndexRange getRangeDOFVariable(const BerdyDynamicVariablesTypes dynamicVariableType, const DOFIndex idx) const;

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
     * Extract the joint torques from the dynamic variables
     */
    bool extractJointTorquesFromDynamicVariables(const VectorDynSize& d,
                                                 const VectorDynSize& jointPos,
                                                       VectorDynSize& jointTorques) const;

    /**
     * Extract the net external force-torques from the dynamic variables
     */
    bool extractLinkNetExternalWrenchesFromDynamicVariables(const VectorDynSize& d,
                                                            LinkNetExternalWrenches& netExtWrenches) const;


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
     * @param[in] gravity the gravity acceleration vector, expressed in the specified fixed frame.
     *
     * \note gravity is used only if selected BerdyVariant is ORIGINAL_BERDY_FIXED_BASE.
     *
     * @return true if all went ok, false otherwise.
     *
     * @note this is equivalent to 
     * @code updateKinematicsFromFixedBase(jointPos,jointVel,m_dynamicsTraversal.getBaseLink()->getIndex(),gravity);
     * @endcode
     *
     */
    bool updateKinematicsFromTraversalFixedBase(const JointPosDoubleArray  & jointPos,
                                                const JointDOFsDoubleArray & jointVel,
                                                const Vector3 & gravity);

    //@}

    /**
     * Set/get the transformation link_H_contact between the link frame and the frame in which the measured net ext wrench is expressed.
     *
     * The default value is the identity. This is extremly useful to correctly tune the variances when only a subset of the external
     * net wrench is known (for example when it is known that the external net wrench is a pure force on a point different from the
     * link frame.
     *
     * \note This will only change the frame in which the measurent equation of the net external wrench is expressed,
     *       not how the Newton-Euler equation of the link are expressed or how the net ext wrenches are serialized in
     *       the LinkNetExternalWrenches class.
     */
    ///@{
    bool setNetExternalWrenchMeasurementFrame(const LinkIndex lnkIndex, const Transform& link_H_contact);
    bool getNetExternalWrenchMeasurementFrame(const LinkIndex lnkIndex, Transform& link_H_contact) const;
    ///@}


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
