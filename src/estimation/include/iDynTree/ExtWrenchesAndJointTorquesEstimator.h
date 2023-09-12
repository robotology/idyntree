// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_ESTIMATION_EXTWRENCHESANDJOINTTORQUEESTIMATOR_H
#define IDYNTREE_ESTIMATION_EXTWRENCHESANDJOINTTORQUEESTIMATOR_H

#include <iDynTree/ExternalWrenchesEstimation.h>

#include <iDynTree/Model.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/ContactWrench.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/JointState.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/LinkTraversalsCache.h>

#include <iDynTree/Sensors.h>

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
 *
 * Beside its main goal of estimation of external wrenches and joint torques, the class
 * also provide methods that can be useful to calibrate the six-axis FT sensors of the robot.
 * These methods are:
 *   * ExtWrenchesAndJointTorquesEstimator::computeExpectedFTSensorsMeasurements
 *   * ExtWrenchesAndJointTorquesEstimator::computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics
 * Details of each method can be found in the method documentation
 */
class ExtWrenchesAndJointTorquesEstimator
{
    /**
     * Structure variables.
     */
    Model m_model;
    SubModelDecomposition m_submodels;
    bool m_isModelValid;
    bool m_isKinematicsUpdated;

    /**< Traveral used for the dynamics computations */
    Traversal m_dynamicTraversal;

    /**
     * Vector of Traversal used for the kinematic computations.
     * m_kinematicTraversals.getTraversalWithLinkAsBase(l) contains the traversal with base link l .
     */
    LinkTraversalsCache m_kinematicTraversals;

    JointPosDoubleArray m_jointPos;
    JointDOFsDoubleArray m_jointVel;
    JointDOFsDoubleArray m_jointAcc;
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
   ExtWrenchesAndJointTorquesEstimator(const ExtWrenchesAndJointTorquesEstimator & /*other*/) {};

   /**
    * Copy operator is forbidden
    */
   ExtWrenchesAndJointTorquesEstimator& operator=(const ExtWrenchesAndJointTorquesEstimator &/*other*/) {return *this;};

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
     * @return true if all went well (model and sensors are well formed), false otherwise.
     */
    bool setModel(const Model & _model);

    /**
     * \brief Set model and sensors used for the estimation.
     *
     * @param[in] _model the kinematic and dynamic model used for the estimation.
     * @param[in] _sensors the sensor model used for the estimation.
     * @return true if all went well (model and sensors are well formed), false otherwise.
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Deprecated, please use variant of this method (i.e. setModel) where SensorsList is passed via the iDynTree::Model.")
    bool setModelAndSensors(const Model & _model, const SensorsList & _sensors);

    /**
     * Load model and sensors from file.
     *
     * @deprecated Use iDynTree::ModelLoader::loadModelFromFile and call setModelAndSensors
     * on the parsed Model and SensorsList
     *
     * @param[in] filename path to the file to load.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     */
    bool loadModelAndSensorsFromFile(const std::string filename, const std::string filetype="");

    /**
     * Load model and sensors from file, specifieng the dof considered for the estimation.
     *
     * @note this will create e a reduced model only with the joint specified in consideredDOFs and  the
     *       fixed joints in which FT sensor are mounted.
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredDOFs list of dof to consider in the model.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     */
    bool loadModelAndSensorsFromFileWithSpecifiedDOFs(const std::string filename,
                                                      const std::vector<std::string> & consideredDOFs,
                                                      const std::string filetype="");


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
    IDYNTREE_DEPRECATED_WITH_MSG("Deprecated, please use model::sensors.")
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
    bool updateKinematicsFromFloatingBase(const VectorDynSize  & jointPos,
                                          const VectorDynSize & jointVel,
                                          const VectorDynSize & jointAcc,
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
    bool updateKinematicsFromFixedBase(const VectorDynSize  & jointPos,
                                       const VectorDynSize & jointVel,
                                       const VectorDynSize & jointAcc,
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
     * \warning Before calling this method, either updateKinematicsFromFloatingBase or
     *          updateKinematicsFromFixedBase must be called.
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
     * \brief For each submodel without any external wrench, computes the equation that relates the FT sensor measures with the kinematics-related known terms.
     *
     * For each submodel in which there are no external forces, we can write the following equation:
     *
     * \f[
     *  A w = b
     * \f]
     * Where:
     *   * \f$w\f$ is the vector of dimension 6*nrOfFTSensors obtained by stacking the FT sensors measures:
     *   * \f$A\f$ is a matrix of size 6 x 6*nrOfFTSensors that depends on position in space of the FT sensors
     *   * \f$b\f$ is a vector of size 6 that depends on the position, velocity, acceleration and gravity of each link in the submodel
     *
     * This function provides an easy way to compute A and B . Typically, these quantities are not used online
     * during the estimation of external wrenches or  internal torques, but rather as an helper method when calibrating FT sensors.
     *
     * In the rest of the documentation, we will refer to this quantities:
     *   * nrOfSubModels (\f$n_{sm}\f$): the number of submodels in which the model is divided, as induced by the FT sensors present in the model.
     *   * nrOfSubModelsWithoutExtWrenches: the number of submodels on which there is no external wrench
     *   * nrOfFTSensors (\f$n_{ft}\f$): the number of FT sensors present in the model
     * In particular, the value of A and b for a given submodel is the following. First of all, for any submodel $sm$ with no external force,
     * we can write (from Equation 4.19 of https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf, modified to account
     * for all FT sensors in the submodel and to remove external wrenches):
     *
     * \f[
     *   \sum_{s=1}^{n_ft}~\mu_{sm, ft}~\sigma_{sm, ft}~{}_{B_{sm}} X^{ft} \mathrm{f}^{meas}_{ft} = \sum_{L \in \mathbb{L}_{sm}} {}_{B_{sm}} X^{L} {}_L \phi_L
     * \f]
     *
     * where:
     *  * \f$ \mu_{sm, ft} \f$ is equal to \f$1\f$ if the sensor \f$ft\f$ is attached to the submodel \f$sm\f$, and \f$0\f$ otherwise
     *  * \f$ \sigma_{sm, ft} \f$ is equal to \f$1\f$ if the sensor \f$ft\f$ is measuring the force applied on submodel \f$sm\f$ or \f$-1\f$ if it is measuring the force that the submodel excerts on its neighbor submodel
     *  *\f$ {B_{sm}} \f$ is a frame in which this equation is expressed, that for this function it is the base link of the submodel.
     *
     *
     * With this definitions, we can see that A_sm and b_sm for a given submodel \f$sm\f$ can be simply be defined as:
     *
     * \f$
     * A_{sm} = \begin{bmatrix} \mu_{sm, ft0}~\sigma_{sm, ft(0)}~{}_C X^{ft(0)}  & \hdots & \mu_{sm, ft(n_ft-1)}~\sigma_{sm, ft(n_ft-1)}~X^{ft(n_ft-1)} \end{bmatrix}
     * \f$
     *
     * \f$
     * w_{sm} = \begin{bmatrix} \mathrm{f}^{meas}_{ft(0)}  \\ \vdots \\ \mathrm{f}^{meas}_{ft(n_ft-1)}  \end{bmatrix}
     * \f$
     *
     * \f$
     * b_{sm} = \sum_{L \in \mathbb{L}_{sm}} {}_C X^{L} {}^L \phi_L
     * \f$
     *
     * \warning Before calling this method, either updateKinematicsFromFloatingBase or
     *          updateKinematicsFromFixedBase must be called.
     *
     * @param[in] unknowns the unknown external wrenches, that is used to understand the submodels in which no external wrench is present
     * @param[out] A vector of nrOfSubModelsWithoutExtWrenches matrices of size 6 x 6*nrOfFTSensors
     * @param[out] b vector of nrOfSubModelsWithoutExtWrenches vectors of size 6
     * @param[out] subModelIDs vector of size nrOfSubModelsWithoutExtWrenches of unsigned integers from 0 to nrOfSubModels-1, subModelIDs[i] specifies to which submodel the quantities A[i]
     * @param[out] baseLinkIndeces vector of size nrOfSubModelsWithoutExtWrenches of iDynTree::LinkIndex from 0 to nrOfLinks-1, baseLinkIndeces[i] specifies the link in which the equation i is expressed
     * @return true if all went well, false otherwise.
     */
    bool computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics(const LinkUnknownWrenchContacts & unknowns,
                                                                     std::vector<iDynTree::MatrixDynSize>& A,
                                                                     std::vector<iDynTree::VectorDynSize>& b,
                                                                     std::vector<std::ptrdiff_t>& subModelID,
                                                                     std::vector<iDynTree::LinkIndex>& baseLinkIndeces);

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
     * \note There should be at least 6 unknowns variables for each submodel for which the estimation of
     *       external wrenches is performed (i.e. usually 1 unknown wrench for each submodel).
     *       If is not the case, undefined results could occur in the estimation.
     *
     * @param[in] unknowns the unknown external wrenches
     * @param[in] ftSensorsMeasures the measurements for the FT sensors.
     * @param[out] estimatedContactWrenches the estimated contact wrenches.
     * @param[out] estimatedJointTorques the estimated joint torques.
     * @return true if all went ok, false otherwise.
     */
    bool estimateExtWrenchesAndJointTorques(const LinkUnknownWrenchContacts & unknowns,
                                            const SensorsMeasurements & ftSensorsMeasures,
                                                  LinkContactWrenches & estimatedContactWrenches,
                                                  JointDOFsDoubleArray & estimatedJointTorques);

    /**
     * Check if the kinematics set in the model are the one of a fixed model.
     *
     * While computing the expected F/T sensors measures, you tipically want the
     * model to be still, to reduce the sources of noise .
     *
     * @param[in] gravityNorm the norm of the gravity (tipically 9.81) against with all the
     *                        proper accelerations are check (for a still model, the proper
     *                        acceleration norm should be close to the gravity norm.
     * @param[in] properAccTol tolerance to use for the check on the proper acceleration norm.
     * @param[in] verbose     true if you want to print debug information, false otherwise.
     * @return true if the model is still, false if it is moving or if the kinematics was never setted.
     *
     * \note This method can be computationally expensive, so in most case it
     *       may be a better idea to just do the check on the input variables (joint velocities, joint acceleration).
     */
    bool checkThatTheModelIsStill(const double gravityNorm,
                                  const double properAccTol,
                                  const double verbose);


    /**
     * Compute the vector of the sum of all the wrenches (both internal and external, excluding gravity) acting on
     * link i, expressed (both orientation and point) with respect to the reference frame of link i,
     * using the articulated body model and the kinematics information provided by the updateKinematics* methods.
     *
     * This is tipically computed as I*a+v*(I*v) , where a is the proper acceleration.
     *
     * @param[out] netWrenches the vector of link net wrenches.
     * @return true if all went ok, false otherwise.
     */
    bool estimateLinkNetWrenchesWithoutGravity(LinkNetTotalWrenchesWithoutGravity & netWrenches);

};

}

#endif
