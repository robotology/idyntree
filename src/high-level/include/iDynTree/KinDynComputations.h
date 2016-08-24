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

namespace iDynTree
{

class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;
class SpatialInertia;
class ClassicalAcc;
class SpatialAcc;
class Wrench;
class Model;
class Traversal;


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

    // Invalidate the cache of intermediated results (called by setRobotState
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
     * Get the used FrameVelocityConvention.
     */
    bool setFrameVelocityRepresentation(const FrameVelocityConvention ) const;

    /**
     * Get the used FrameVelocityConvention.
     */
    FrameVelocityConvention getFrameVelocityRepresentation() const;
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
     * Set the state for the robot (floating base)
     *
     * @param q a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param q_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_T_base  the homogeneous transformation that transforms position vectors expressed in the base reference frame
     *                      in position frames expressed in the world reference frame (i.e. pos_world = world_T_base*pos_base .
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed with the convention specified by the used FrameVelocityConvention.
     *
     *
     */
    bool setRobotState(const iDynTree::Transform &world_T_base,
                       const iDynTree::VectorDynSize& q,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::VectorDynSize& q_dot,
                       const iDynTree::Vector3& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *
     */
    bool setRobotState(const iDynTree::VectorDynSize &q,
                       const iDynTree::VectorDynSize &q_dot,
                       const iDynTree::Vector3& world_gravity);

    /**
     * Access the robot state.
     */
    iDynTree::Transform getWorldBaseTransform();
    iDynTree::Twist     getBaseTwist();

    bool getJointPos(iDynTree::VectorDynSize &q);
    bool getJointVel(iDynTree::VectorDynSize &dq);

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

    bool getFrameJacobian(const std::string & frameName,
                          iDynTree::MatrixDynSize & outJacobian) const;

    bool getFrameJacobian(const unsigned int & frameIndex,
                          iDynTree::MatrixDynSize & outJacobian) const;

    //@}


};

}

#endif

