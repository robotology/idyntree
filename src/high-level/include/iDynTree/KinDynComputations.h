/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_KINDYNCOMPUTATIONS_H
#define IDYNTREE_KINDYNCOMPUTATIONS_H

#include <string>

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{

class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;
class SpatialInertia;
class SpatialMomentum;
class ClassicalAcc;
class SpatialAcc;
class Wrench;
class Model;
class Traversal;
class Position;
class FreeFloatingGeneralizedTorques;

/**
 * \ingroup iDynTreeHighLevel
 *
 * \brief High level stateful class wrapping several kinematics and dynamics algorithms.
 *
 * The kinematics dynamics computations class is an high level class stateful to access
 * several algorithms related to kinematics and dynamics of free floating robot systems.
 *
 * This class supports three possible convention to express the floating base information :
 * the inertial, the body-fixed and the mixed convention.
 * To get more info on this three conventions, check section II.C  of
 * On  the  Base  Frame  Choice  in  Free-Floating  Mechanical  Systems
 * and  its  Connection  to  Centroidal  Dynamics
 * https://traversaro.github.io/preprints/changebase.pdf
 *
 */
class KinDynComputations {
private:
    struct KinDynComputationsPrivateAttributes;
    KinDynComputationsPrivateAttributes * pimpl;

    // copy is disabled for the moment
    KinDynComputations(const KinDynComputations & other);
    KinDynComputations& operator=(const KinDynComputations& other);

    // Make sure that (if necessary) forward kinematics is updated
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeFwdKinematics();

    // Make sure that (if necessary) mass matrix is updated
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeRawMassMatrixAndTotalMomentum();

    // Make sure that (if necessary) the kinematics bias acc is update
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeBiasAccFwdKinematics();

    // Invalidate the cache of intermediated results (called by setRobotState)
    void invalidateCache();

    // Resize internal data structures after a model has been successfully loaded
    void resizeInternalDataStructures();
public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    KinDynComputations();


    /**
     * Destructor
     *
     */
    virtual ~KinDynComputations();
    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to load the structure of your model.
     */
    //@{

    /**
     * Load the model of the robot from a iDynTree::Model class.
     *
     * @param model the model to use in this class.
     * @return true if all went ok, false otherwise.
     */
    bool loadRobotModel(const iDynTree::Model & model );

    /**
     * Load the model of the robot from an external file.
     *
     * @param filename path to the file to load
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadRobotModelFromFile(const std::string & filename, const std::string & filetype="urdf");

    /**
     * Load the model of the robot  from a string.
     *
     * @param filename string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadRobotModelFromString(const std::string & modelString, const std::string & filetype="urdf");

    /**
     * Return true if the models for the robot have been correctly.
     *
     * @return True if the class has been correctly configure, false otherwise.
     */
    bool isValid() const;

    /**
     * Set the used FrameVelocityRepresentation.
     */
    bool setFrameVelocityRepresentation(const FrameVelocityRepresentation framVelRepr) const;

    /**
     * Get the used FrameVelocityRepresentation.
     */
    FrameVelocityRepresentation getFrameVelocityRepresentation() const;
    //@}


    /**
     * Get the number of internal degrees of freedom of the robot model used
     * in the class.
     * This return the *internal* degrees of freedom, because it does not include
     * the eventual 6 degrees of freedom usually associated with the floating base.
     *
     * @return the number of internal degrees of freedom of the model.
     */
    unsigned int getNrOfDegreesOfFreedom() const;

    /**
     * Get a human readable description of a given internal degree of freedom of the robot.
     *
     */
    std::string getDescriptionOfDegreeOfFreedom(int dof_index);

    /**
     * Get a human readable description of all the internal degrees of freedom of the robot.
     *
     * @return a std::string containing the description of the internal degrees of freedom.
     */
    std::string getDescriptionOfDegreesOfFreedom();


    /**
     * Get the number of links contained in the model.
     *
     */
    unsigned int getNrOfLinks() const;

    /**
     * Get a human readable description of a given link in the model.
     *
     * @return a human readable description of a given link in the model.
     */
    //std::string getDescriptionOfLink(int link_index);

    /**
     * Get a human readable description of all links considered in the model.
     *
     * @return a std::string containing the description of all the links.
     */
    //std::string getDescriptionOfLinks();

    /**
     * Get the number of frames contained in the model.
     *
     * \note The number of frames is always greater than or equal to
     *       the number of links because every link has one link reference frame
     *       associated. An arbitrary number of additional reference frames
     *       can associated to a given link.
     */
    unsigned int getNrOfFrames() const;

    /**
     * Get a human readable description of a given frame considered in the model.
     *
     * @return a human readable description of a given frame considered in the model.
     */
    //std::string getDescriptionOfFrame(int frame_index);

    /**
     * Get a human readable description of all frames considered in the model.
     *
     * @return a std::string containing the description of all the frame considered in the model.
     */
    //std::string getDescriptionOfLinks();

    /**
     * Get the name of the link considered as the floating base.
     *
     * @return the name of the base link.
     */
    std::string getFloatingBase() const;

    /**
     * Set the link that is used as the floating base link.
     *
     *
     * @return true if all went well, false otherwise (for example if the link name was not found).
     */
    bool setFloatingBase(const std::string & floatingBaseName);


    //@}


    /**
     * @name Methods to access the underlyng model for the robot
     */
    //@{

    const Model & model() const;
    const Model & getRobotModel() const;

    //@}

    /**
      * @name Methods to submit the input data for dynamics computations.
      */
    //@{

    /**
     * @brief Set the (internal) joint positions.
     *
     * \note This method sets only the joint positions, leaving all the other components of the state to their previous value.
     *
     * @param[in] s A vector of dimension this->model().getNrOfPosCoords() .
     * @return true if all went well, false otherwise.
     */
    bool setJointPos(const iDynTree::VectorDynSize& s);

    /**
     * Set the state for the robot (floating base)
     *
     * @param world_T_base  the homogeneous transformation that transforms position vectors expressed in the base reference frame
     *                      in position frames expressed in the world reference frame (i.e. pos_world = world_T_base*pos_base .
     * @param s a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed with the convention specified by the used FrameVelocityConvention.
     * @param s_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_gravity a 3d vector of the gravity acceleration vector, expressed in the world/inertial frame.
     *
     */
    bool setRobotState(const iDynTree::Transform &world_T_base,
                       const iDynTree::VectorDynSize& s,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::VectorDynSize& s_dot,
                       const iDynTree::Vector3& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *
     */
    bool setRobotState(const iDynTree::VectorDynSize &s,
                       const iDynTree::VectorDynSize &s_dot,
                       const iDynTree::Vector3& world_gravity);

    /**
     * Access the robot state.
     */
    iDynTree::Transform getWorldBaseTransform();
    iDynTree::Twist     getBaseTwist();

    bool getJointPos(iDynTree::VectorDynSize &q);
    bool getJointVel(iDynTree::VectorDynSize &dq);

    /**
     * Get the n+6 velocity of the model.
     * Obtained by stacking the output of getBaseTwist and of getJointVel .
     */
    bool getModelVel(iDynTree::VectorDynSize &nu);

    //@}

    /**
      * @name Methods to get transform information between frames in the model,
      *       given the current state.
      */
    //@{

    /**
     * Get the index corresponding to a given frame name.
     * @return a integer greater than or equal to zero if the frame exist,
     *         a negative integer otherwise.
     */
    int getFrameIndex(const std::string & frameName) const;

    /**
     * Get the frame name corresponding to a given frame index.
     *
     */
    std::string getFrameName(const iDynTree::FrameIndex frameIndex) const;

    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the world one
     * (world_H_frame).
     *
     */
    iDynTree::Transform getWorldTransform(const iDynTree::FrameIndex frameIndex);

    /**
     * Version of getWorldTransform where the frame is specified by name.
     *
     * @note For real time code use the method that takes an integer.
     *
     */
    iDynTree::Transform getWorldTransform(std::string frameName);


    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the one specified
     * by refFrameIndex (refFrame_H_frame).
     *
     */
    iDynTree::Transform getRelativeTransform(const iDynTree::FrameIndex refFrameIndex,
                                             const iDynTree::FrameIndex frameIndex);

     /**
     * Return the transform between the frame with the origin of the frameOriginIndex
     * and the orientation of frameOrientationIndex and the one with the origin
     * of refFrameOriginIndex and the orientation of refFrameOrientationIndex .
     *
     * In symbols we return the (refFrameOrigin,refFrameOrientation)_H_(frameOrigin,frameORientation)
     *
     * This is a variant of the getRelativeTransform in which the orientation and origin part of both
     * side of the transform are explicited.
     *
     * \todo provide mode detailed documentation.
     *
     */
    iDynTree::Transform getRelativeTransformExplicit(const iDynTree::FrameIndex refFrameOriginIndex,
                                                     const iDynTree::FrameIndex refFrameOrientationIndex,
                                                     const iDynTree::FrameIndex    frameOriginIndex,
                                                     const iDynTree::FrameIndex    frameOrientationIndex);

    /**
     * Version of getRelativeTransform where the frames are specified by name.
     *
     * @note For real time code use the method that takes an integer.
     *
     */
    iDynTree::Transform getRelativeTransform(const std::string & refFrameName,
                                             const std::string & frameName);

    //@}

    /**
      * @name Methods to get frame velocity information given the current state.
      */
    //@{

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation .
     */
    iDynTree::Twist getFrameVel(const std::string & frameName);

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation .
     */
    iDynTree::Twist getFrameVel(const FrameIndex frameIdx);

    bool getFrameFreeFloatingJacobian(const std::string & frameName,
                                      iDynTree::MatrixDynSize & outJacobian);

    bool getFrameFreeFloatingJacobian(const FrameIndex frameIndex,
                                      iDynTree::MatrixDynSize & outJacobian);

    /**
     * Get the bias acceleration (i.e. acceleration not due to robot acceleration) of the frame velocity.
     *
     * This term is usually called $\dot{J} \nu$ or $\dot{J} \dot{q}$.
     */
    Vector6 getFrameBiasAcc(const FrameIndex frameIdx);

    /**
     * Get the bias acceleration (i.e. acceleration not due to robot acceleration) of the frame velocity.
     *
     * This term is usually called $\dot{J} \nu$ or $\dot{J} \dot{q}$.
     */
    Vector6 getFrameBiasAcc(const std::string & frameName);


    // Todo getFrameRelativeVel and getFrameRelativeJacobian to match the getRelativeTransform behaviour

    //@}


    /**
      * @name Methods to get quantities related to centroidal dynamics.
      *
      * For a precise definition of the quantities computed by this methods, please check:
      * S. Traversaro, D. Pucci, F. Nori
      * On the Base Frame Choice in Free-Floating Mechanical Systems and its Connection to Centroidal Dynamics
      * https://traversaro.github.io/preprints/changebase.pdf
      */
    //@{

    /**
     * Return the center of mass position.
     *
     * @return the center of mass position, expressed in the world/inertial frame.
     */
    iDynTree::Position getCenterOfMassPosition();

    /**
     * Return the center of mass velocity, with respect to the world/inertial frame.
     *
     * \note This is the time derivative of the quantity returned by getCenterOfMassPosition .
     *
     */
    iDynTree::Vector3 getCenterOfMassVelocity();

    /**
     * Return the center of mass jacobian, i.e. the 3 \times (n+6) matrix such that:
     *  getCenterOfMassVelocity() == getCenterOfMassJacobian() * \nu .
     *
     */
    bool getCenterOfMassJacobian(MatrixDynSize & comJacobian);

    /**
     * Return the center of mass bias acceleration.
     */
     Vector3 getCenterOfMassBiasAcc();

    /**
     * Get the average velocity of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note the linear part of this twist correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    iDynTree::Twist getAverageVelocity();

    /**
     * Get the jacobian of the average velocity of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note the linear part of this jacobian correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    bool getAverageVelocityJacobian(MatrixDynSize & avgVelocityJacobian);

    /**
     * Get the centroidal average velocity of the robot.
     *
     * The quantity is the average velocity returned by getAverageVelocity, but computed in the center of mass
     * and with the orientation of the FrameVelocityRepresentation used.
     * It we indicate with G the center of mass, it is expressed in (G[A]) for the mixed and inertial representation,
     * and in (G[B]) for the base body-fixed representation.
     *
     * \note the linear part of this twist correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED or INERTIAL.
     *
     */
    iDynTree::Twist getCentroidalAverageVelocity();

    /**
     * Get the jacobian of the centroidal average velocity of the robot.
     *
     * See the getCentroidalAverageVelocity method for more info on this.
     */
    bool getCentroidalAverageVelocityJacobian(MatrixDynSize & centroidalAvgVelocityJacobian);

    /**
     * Get the linear and angular momentum of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    iDynTree::SpatialMomentum getLinearAngularMomentum();

    /**
     * Get the linear and angular momentum of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    bool getLinearAngularMomentumJacobian(MatrixDynSize & linAngMomentumJacobian);

    /**
     * Get the centroidal (total) momentum of the robot.
     * If G is the center of mass, this quantity is expressed in (G[A]), (G[A]) or (G[B]) depending on the FrameVelocityConvention used.
     *
     */
    iDynTree::SpatialMomentum getCentroidalTotalMomentum();

    //@}


    /**
      * @name Methods to get quantities related to dynamics matrices.
      */
    //@{

    /**
     * Get the free floating mass matrix of the system.
     *
     * The mass matrix depends on the joint positions, specified by the setRobotState methods.
     * If the chosen FrameVelocityRepresentation is MIXED_REPRESENTATION or INERTIAL_FIXED_REPRESENTATION,
     * the mass matrix depends also on the base orientation with respect to the inertial frame,
     * that is also set by the  setRobotState methods.
     *
     * For more details on the structure of the free floating mass matrix, please check:
     * S. Traversaro, A. Saccon
     * Multibody Dynamics Notation
     * http://repository.tue.nl/849895
     *
     * @param[out] freeFloatingMassMatrix the (6+getNrOfDOFs()) times (6+getNrOfDOFs()) output mass matrix.
     * @return true if all went well, false otherwise.
     */
    bool getFreeFloatingMassMatrix(MatrixDynSize & freeFloatingMassMatrix);

    //@}

    /**
      * @name Methods to unconstrained free floating dynamics.
      */
    //@{

    /**
     * Compute the free floating inverse dynamics.
     *
     * The semantics of baseAcc, the base part of baseForceAndJointTorques
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[in] baseAcc the acceleration of the base link
     * @param[in] s_ddot the accelerations of the joints
     * @param[in] linkExtForces the external wrenches excerted by the environment on the model
     * @param[out] baseForceAndJointTorques the output generalized torques
     * @return true if all went well, false otherwise
     */
    bool inverseDynamics(const Vector6& baseAcc,
                         const VectorDynSize& s_ddot,
                         const LinkNetExternalWrenches & linkExtForces,
                               FreeFloatingGeneralizedTorques & baseForceAndJointTorques);

    /**
     * Compute the getNrOfDOFS()+6 vector of generalized bias (gravity+coriolis) forces.
     *
     * The semantics of baseAcc, the base part of baseForceAndJointTorques
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[in] linkExtForces the external wrenches excerted by the environment on the model
     * @param[out] baseForceAndJointTorques the output generalized bias forces
     * @return true if all went well, false otherwise
     */
    bool generalizedBiasForces(FreeFloatingGeneralizedTorques & generalizedGravityForces);

    /**
     * Compute the getNrOfDOFS()+6 vector of generalized gravity forces.
     *
     * The semantics of baseAcc, the base part of baseForceAndJointTorques
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] baseForceAndJointTorques the output gravity generalized forces
     * @return true if all went well, false otherwise
     */
    bool generalizedGravityForces(FreeFloatingGeneralizedTorques & generalizedGravityForces);

    //@}


};

}

#endif

