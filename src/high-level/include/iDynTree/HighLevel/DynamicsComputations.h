/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNCOMPUTATIONS_H

#include <string>

namespace iDynTree {

class Position;
class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;
class SpatialInertia;
class ClassicalAcc;
class SpatialAcc;
class Wrench;

namespace HighLevel {

/**
 * \ingroup iDynTreeHighLevel
 *
 * The dynamics computations class is an high level class stateful to access
 * several algorithms related to kinematics and dynamics of free floating
 * robot systems.
 *
 */
class DynamicsComputations {
private:
    struct DynamicsComputationsPrivateAttributes;
    DynamicsComputationsPrivateAttributes * pimpl;

    // copy is disabled for the moment
    DynamicsComputations(const DynamicsComputations & other);
    DynamicsComputations& operator=(const DynamicsComputations& other);

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
    DynamicsComputations();


    /**
     * Destructor
     *
     */
    virtual ~DynamicsComputations();
    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to load the structure of your model.
     */
    //@{

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
     * Return true if the models for the robot, sensors and regressors have been correctly
     * loaded and the regressor generator is ready to compute regressors.
     *
     * @return True if the regressor generator is correctly configure, false otherwise.
     */
    bool isValid();
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
     * \note The number of frames is always greater or equal then
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
     * (todo)
     */
    //@{

    //const UndirectedTree & getRobotModel();

    //@}

    /**
      * @name Methods to submit the input data for dynamics computations.
      */
    //@{

    /**
     * Set the state for the robot (floating base)
     *
     * @param qj a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param qj_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param qj_dotdot a vector of getNrOfDegreesOfFreedom() joint acceleration (in rad/sec^2)
     * @param world_T_base  the homogeneous transformation that transforms position vectors expressed in the base reference frame
     *                      in position frames expressed in the world reference frame (i.e. pos_world = world_T_base*pos_base .
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed in the world orientation frame and with respect
     *                      to the base origin.
     * @param base_acceleration The 6d classical acceleration (linear/angular acceleration) of the base
     *                          expressed in the world orientation frame and with respect to the base origin.
     *                          Plese note that this is *not* the spatial acceleration of the base link.
     *                          For more info check http://wiki.icub.org/codyco/dox/html/dynamics_notation.html .
     *
     * \note this convention is the same used in the wholeBodyInterface classes.
     *
     * \todo TODO define a proper RobotState class, so we centralize all this definitions and we can leave
     *            to the user the freedom of setting spatial/angular acceleration and the reference frames in
     *            which to express quantities.
     */
    bool setRobotState(const iDynTree::VectorDynSize& q,
                       const iDynTree::VectorDynSize& q_dot,
                       const iDynTree::VectorDynSize& q_dotdot,
                       const iDynTree::Transform &world_T_base,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::ClassicalAcc& base_acceleration,
                       const iDynTree::SpatialAcc& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *  base_acceleration = iDynTree::Twist::Zero();
     *
     * \todo TODO define a RobotState class.
     */
    bool setRobotState(const iDynTree::VectorDynSize &q,
                       const iDynTree::VectorDynSize &q_dot,
                       const iDynTree::VectorDynSize &q_dotdot,
                       const iDynTree::SpatialAcc& world_gravity);

    /**
     *
     *
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
     * @return a integer greater or equal then zero if the frame exist,
     *         a negative integer otherwise.
     */
    int getFrameIndex(const std::string & frameName) const;

    /**
     * Get the frame name corresponding to a given frame index.
     *
     */
    std::string getFrameName(int frameIndex) const;

    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the world one (world_H_frame).
     *
     */
    iDynTree::Transform getWorldTransform(unsigned int frameIndex);

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
    iDynTree::Transform getRelativeTransform(unsigned int refFrameIndex,
                                             unsigned int frameIndex);

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
      * @name Methods to get velocity and acceleration information of the frames in the model,
      *       given the current state.
      */
    //@{

    /**
     * Given the current state, return the twist of frame of name frameName,
     * expressed with respect to the origin of frame frameName and using
     * the orientation of frameName.
     */
    iDynTree::Twist getFrameTwist(const std::string & frameName);

    /**
     * Given the current state, return the twist of frame of index frameIndex,
     * expressed with respect to the origin of frameIndex and using
     * the orientation of frameIndex .
     */
    iDynTree::Twist getFrameTwist(const int frameIndex);

    /**
     * Given the current state, return the twist of frame of name frameName,
     * expressed with respect to the origin of frame frameName and using
     * the world orientation .
     */
    iDynTree::Twist getFrameTwistInWorldOrient(const std::string & frameName);

    /**
     * Given the current state, return the twist of frame of index frameIndex,
     * expressed with respect to the origin of frameIndex and using
     * the world orientation .
     */
    iDynTree::Twist getFrameTwistInWorldOrient(const int frameIndex);

    /**
     * Get the proper classical acceleration (i.e. the actual acceleration of the link minus
     * the gravitational acceleration) of frame of name frameName,
     * expressed with respect to the origin of frameIndex and using
     * the orientation of frameIndex .
     *
     */
    /* iDynTree::ClassicalAcc getFrameProperClassicalAcceleration(const std::string & frameName);
                                                                 */

    /**
     * Get the proper classical acceleration (i.e. the actual acceleration of the link minus
     * the gravitational acceleration) of frame of index frameIndex,
     * expressed with respect to the origin of pointFrameIndex and using
     * the orientation of orientationFrameIndex .
     *
     */
    /*
    iDynTree::ClassicalAcc getFrameProperClassicalAcceleration(const int frameIndex);*/


    /**
     * Get the proper spatial acceleration (i.e. the actual acceleration of the link minus
     * the gravitational acceleration) of frame of name frameName,
     * expressed with respect to the origin of frameName and using
     * the orientation of frameName  .
     *
     */
    iDynTree::SpatialAcc getFrameProperSpatialAcceleration(const std::string & frameName);


    /**
     * Get the proper classical acceleration (i.e. the actual acceleration of the link minus
     * the gravitational acceleration) of frame of index frameIndex,
     * expressed with respect to the origin of frameIndex and using
     * the orientation of frameIndex .
     *
     * By default, return the acceleration expressed in the frame orientation and with
     * respect to the frame origin.
     */
    iDynTree::SpatialAcc getFrameProperSpatialAcceleration(const int frameIndex);


    //@}


     /**
      * @name Methods to get link information and parameters.
      *       This methods are necessary because we don't have a proper
      *       iDynTree::Model class, and we still rely on KDL::CoDyCo::UndirectedTree
      *       that we cannot exposed directly in pure iDynTree interfaces.
      *       Once we have a iDynTree::Model class, this methods will be
      *       directly exposed in the iDynTree::Model
      */
    //@{


     /**
     * Get the index corresponding to a given link name.
     * @return a integer greater or equal then zero if the frame exist,
     *         a negative integer otherwise.
     */
    int getLinkIndex(const std::string & linkName) const;

    /**
     * Get the spatial inertia of a given link,
     * expressed in the link reference frame (i.e. with the orientation
     * of the link reference frame and with respect to the link reference
     * frame origin.
     */
    iDynTree::SpatialInertia getLinkInertia(const unsigned int linkIndex) const;

    /**
     * Get the spatial inertia of a given link,
     * expressed in the link reference frame (i.e. with the orientation
     * of the link reference frame and with respect to the link reference
     * frame origin.
     */
    iDynTree::SpatialInertia getLinkInertia(const std::string & linkName) const;

    //@}


    /**
     * @name Methods to compute Inverse Dynamics
     *
     */
    bool inverseDynamics(iDynTree::VectorDynSize & outTorques,
                         iDynTree::Wrench & baseReactionForce);

    /**
      * @name Methods to get Jacobians and Dynamics regressor
      */
    //@{

    bool getFrameJacobian(const std::string & frameName,
                          iDynTree::MatrixDynSize & outJacobian) const;

    bool getFrameJacobian(const unsigned int & frameIndex,
                          iDynTree::MatrixDynSize & outJacobian) const;



    bool getDynamicsRegressor(iDynTree::MatrixDynSize & outRegressor);

    /**
     * Return the dynamics parameters related to dynamics regressors.
     *
     * @param[out] values a Vector of size 10*getNrOfLinks ,
     * @return true if all went well, false if there was an error
     */
    bool getModelDynamicsParameters(iDynTree::VectorDynSize & values) const;

    //@}


    /**
      * @name Center of Mass related methods.
      */
    //@{

    /**
     * @brief getCenterOfMass return the center of mass of the model.
     * @return the center of mass of the model, expressed in the world frame.
     *
     * \note to get the center of mass in any other frame of the robot,
     *       just compute the transform between that frame and the world
     *       and use if to transform the center of mass, as in:
     *       dynComp.getWorldTransform("frameName").inverse()*getCenterOfMass();
     */
    iDynTree::Position getCenterOfMass();

    /**
     * @brief get the 3x(6+getNrOfDegreesOfFreedom()) jacobian of the center of mass
     *        position expressed in the world frame.
     * @param[out] outJacobian a 3x(6+getNrOfDegreesOfFreedom()) matrix, jacobian of the COM.
     * @return true if all wen well, false if there was an error.
     */
    bool getCenterOfMassJacobian(iDynTree::MatrixDynSize & outJacobian);

    //@}
};

}

}


#endif

