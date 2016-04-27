/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
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

#ifndef IDYNTREE_ESTIMATION_EXTWRENCHESANDJOINTTORQUEESTIMATOR_H
#define IDYNTREE_ESTIMATION_EXTWRENCHESANDJOINTTORQUEESTIMATOR_H

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/ContactWrench.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/FreeFloatingState.h>

namespace iDynTree
{

/**
 * \brief Estimator for external wrenches and joint torques using internal F/T sensors.
 *
 * This is a class for estimating external wrenches and joint torques using as an input
 * the robot velocities and the accelerations and the measurement of internal six axis
 * F/T sensors.
 *
 * The kinematic information (position,velocity and acceleration) necessary for the
 * estimation can be provided in two ways:
 * * using the updateKinematicsFromFloatingBase
 * * using the updateKinematicsFromFixedBase
 *
 * Note that in both ways there is no need (for the estimation) to provide the
 * absolute position and linear velocity of the robot with respect to the inertial frame.
 * The effect of gravity is considered by directly using the proper acceleration in the case
 * of the floating frame (proper acceleration can be directly measured by an accelerometer)
 * or by directly providing the gravity vector in the fixed frame case.
 */
class ExtWrenchesAndJointTorquesEstimator
{
    /**
     * Structure variables.
     */
    Model m_model;
    SubModelDecomposition m_submodels;
    SensorsList m_sensors;
    bool m_isModelValid;
    bool m_isKinematicsUpdated;

    /**< Traveral used for the dynamics computations */
    Traversal m_dynamicTraversal;

    /**
     * Vector of Traversal used for the kinematic computations.
     * m_kinematicTraversals[l] contains the traversal with base link l .
     */
    std::vector<Traversal *> m_kinematicTraversals;

    /**
     * Helper functions for dealing with the kinematic traversal dynamic allocation
     */
    void allocKinematicTraversals(const size_t nrOfLinks);
    void freeKinematicTraversals();

    JointPosDoubleArray m_jointPos;
    LinkVelArray m_linkVels;
    LinkAccArray m_linkProperAccs;
    LinkNetExternalWrenches m_linkNetExternalWrenches;
    LinkInternalWrenches    m_linkIntWrenches;
    FreeFloatingGeneralizedTorques m_generalizedTorques;

    estimateExternalWrenchesBuffers m_calibBufs;
    estimateExternalWrenchesBuffers m_bufs;

    /**
     * Disable copy constructor and copy operator
     */
   ExtWrenchesAndJointTorquesEstimator(const ExtWrenchesAndJointTorquesEstimator & other) {};

   /**
    * Copy operator is forbidden
    */
   ExtWrenchesAndJointTorquesEstimator& operator=(const ExtWrenchesAndJointTorquesEstimator &other) {return *this;};

public:
    /**
     * \brief Constructor.
     */
    ExtWrenchesAndJointTorquesEstimator();

    /**
     * \brief Destructor.
     */
    ~ExtWrenchesAndJointTorquesEstimator();

    /**
     * \brief Set model and sensors used for the estimation.
     *
     * @param[in] _model the kinematic and dynamic model used for the estimation.
     * @param[in] _sensors the sensor model used for the estimation.
     * @return true if all went well (model and sensors are well formed), false otherwise.
     */
    bool setModelAndSensors(const Model & _model, const SensorsList & _sensors);

    /**
     * Load model and sensors from file.
     *
     * @param[in] filename path to the file to load.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     *
     */
    bool loadModelAndSensorsFromFile(const std::string filename, const std::string filetype="");

    /**
     * Get used model.
     *
     * @return the kinematic and dynamic model used for estimation.
     */
    const Model & model() const;

    /**
     * Get used sensors.
     *
     * @return the sensor model used for estimation.
     */
    const SensorsList & sensors() const;
    
    /**
     * Get the used submodel decomposition.
     *
     * @return the used submodel decomposition.
     */
    const SubModelDecomposition & submodels() const;

    /**
     * Set the kinematic information necessary for the force estimation using the
     * acceleration and angular velocity information of a floating frame.
     *
     * \note Tipically the floating frame information comes from an sensor
     *       containing a gyroscope (providing the angular velocity) and an
     *       accelerometer (providing the classical proper acceleration). As
     *       inertial sensors that return angular acceleration exist but are not common,
     *       the angular acceleration is usually obtained by numerical derivation
     *       on the angular velocity measure. In some cases it could make sense
     *       to just neglet the contribution of the floating frame angular acceleration
     *       (setting it to a zero vector), if its impact to the dynamics is marginal.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] jointVel the velocities of the joints of the model.
     * @param[in] jointAcc the accelerations of the joints of the model.
     * @param[in] floatingFrame the index of the frame for which kinematic information is provided.
     * @param[in] properClassicalLinearAcceleration proper (actual acceleration-gravity) classical acceleration
     *                                              of the origin of the specified frame,
     *                                              expressed in the specified frame orientation.
     * @param[in] angularVel angular velocity (wrt to an inertial frame) of the specified floating frame,
     *                       expressed in the specified frame orientation.
     * @param[in] angularAcc angular acceleration (wrt to an inertial frame) of the specified floating frame ,
     *                       expressed in the specified frame orientation.
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromFloatingBase(const JointPosDoubleArray  & jointPos,
                                          const JointDOFsDoubleArray & jointVel,
                                          const JointDOFsDoubleArray & jointAcc,
                                          const FrameIndex & floatingFrame,
                                          const Vector3 & properClassicalLinearAcceleration,
                                          const Vector3 & angularVel,
                                          const Vector3 & angularAcc);

    /**
     * Set the kinematic information necessary for the force estimation assuming that a
     * given frame is not accelerating with respect to the inertial frame.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] jointVel the velocities of the joints of the model.
     * @param[in] jointAcc the accelerations of the joints of the model.
     * @param[in] fixedFrame the index of the frame that is not accelerating with respect to the inertial frame.
     * @param[in] gravity the gravity acceleration vector, expressed in the specified fixed frame.
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromFixedBase(const JointPosDoubleArray  & jointPos,
                                       const JointDOFsDoubleArray & jointVel,
                                       const JointDOFsDoubleArray & jointAcc,
                                       const FrameIndex & fixedFrame,
                                       const Vector3 & gravity);

    /**
     * \brief Predict FT sensors using the knoledge of external wrenches location.
     *
     * This function is used to estimate the expected measurement of the FT sensors.
     * The typical use of this function is to specify only one external unknown wrench
     * in the unknowns parameter and then compute the expected measurements of the FT sensors
     * using the kinematic information specified with an updateKinematics*** method.
     * The location of the single external unknown wrench is the one of the only link
     * that is supporting the weight of the robot.
     *
     * This function can also be used to estimate the FT sensor measurements in case
     * two unknown external wrenches are applied on the robot, if some additional
     * assumption about the simmetry of the robot configuration, the joint torques and
     * the external wrenches can be done.
     *
     * @param[in] unknowns the unknown external wrenches.
     * @param[out] predictedMeasures the estimate measures for the FT sensors.
     * @param[out] estimatedContactWrenches the estimated contact wrenches.
     * @param[out] estimatedJointTorques the estimated joint torques.
     * @return true if all went well, false otherwise.
     */
    bool computeExpectedFTSensorsMeasurements(const LinkUnknownWrenchContacts & unknowns,
                                                    SensorsMeasurements & predictedMeasures,
                                                    LinkContactWrenches & estimatedContactWrenches,
                                                    JointDOFsDoubleArray & estimatedJointTorques);

    /**
     * \brief Estimate the external wrenches and the internal joint torques using the measurement of the internal F/T sensors.
     *
     * This is the main method of the class. The technique implemented
     * in this method is the one described in the paper:
     *
     * "Contact force estimations using tactile sensors and force/torque sensors"
     *
     * Del Prete, A., Natale, L., Nori, F., & Metta, G. (2012).
     * Contact force estimations using tactile sensors and force/torque sensors.
     * URL : http://www.researchgate.net/profile/Andrea_Del_Prete/publication/236152161_Contact_Force_Estimations_Using_Tactile_Sensors_and_Force__Torque_Sensors/links/00b495166e74a5369d000000.pdf
     *
     * @param[in] unknowns the unknown external wrenches.
     * @param[in] ftSensorsMeasures the measurements for the FT sensors.
     * @param[out] estimatedContactWrenches the estimated contact wrenches.
     * @param[out] estimatedJointTorques the estimated joint torques.
     * @return true if all went ok, false otherwise.
     */
    bool estimateExtWrenchesAndJointTorques(const LinkUnknownWrenchContacts & unknowns,
                                            const SensorsMeasurements & ftSensorsMeasures,
                                                  LinkContactWrenches & estimateContactWrenches,
                                                  JointDOFsDoubleArray & jointTorques);

};

}

#endif


