/*!
 * @file InverseKinematics.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */


#ifndef IDYNTREE_INVERSEKINEMATICS_H
#define IDYNTREE_INVERSEKINEMATICS_H

#include <string>
#include <vector>

namespace iDynTree {
    class VectorDynSize;
    class Transform;
    class Position;
    class Rotation;
    class Model;
}


namespace iDynTree {
    class InverseKinematics;

    /*!
     * @brief type of parametrization for the rotation (SO3) element
     */
    enum InverseKinematicsRotationParametrization {
        InverseKinematicsRotationParametrizationQuaternion, /*!< Quaternion parametrization */
        InverseKinematicsRotationParametrizationRollPitchYaw, /*!< Roll Pitch Yaw parametrization */
    };

    inline unsigned sizeOfRotationParametrization(enum InverseKinematicsRotationParametrization rotationParametrization)
    {
        switch (rotationParametrization) {
            case InverseKinematicsRotationParametrizationQuaternion:
                return 4;
            case InverseKinematicsRotationParametrizationRollPitchYaw:
                return 3;
        }

		return 0;
    }

    /*!
     * @brief Specify how to solve for the desired target
     *
     * A target frame can be solved as a constraints 
     * (i.e. if it cannot be obtained the problem is unfeasible)
     * or as a cost (best-effort to reach the target)
     */
    enum InverseKinematicsTreatTargetAsConstraint {
        InverseKinematicsTreatTargetAsConstraintNone = 0, //both as costs
        InverseKinematicsTreatTargetAsConstraintPositionOnly = 1, //position as constraint, rotation as cost
        InverseKinematicsTreatTargetAsConstraintRotationOnly = 1 << 1, //rotation as constraint, position as cost
        InverseKinematicsTreatTargetAsConstraintFull = InverseKinematicsTreatTargetAsConstraintPositionOnly | InverseKinematicsTreatTargetAsConstraintRotationOnly, //both as constraints
    };
}

//TODO: how to handle conflicting requirements
/*!
 * @brief NLP-based Inverse kinematics
 *
 * Given a mechanical structure configuration
 * \f[ q \in SE(3) \times \mathbb{R}^n \f] and
 * possibly multiple target frames
 * \f[ F_i^d \in \SE(3) \f]
 * the inverse kinematics is responsible to find the
 * configuration \f$ q^* \f$ such that
 * \f[ F_i(q^*) = F_i^d \forall i, \f]
 * where the meaning of the \f$=\f$ and \f$\forall\f$
 * depends on the resolution mode and on the references specified
 *
 * Example
 * @code
 * //Allocate an inverse kinematics object
 * iDynTree::InverseKinematric ik;
 * @endcode
 *
 * @note all the cartesian frames must be specified w.r.t. the same global frame.
 * This library does not assume any particular global frame
 *
 *
 *
 */
class iDynTree::InverseKinematics
{

public:
    /*!
     * Default constructor
     */
    InverseKinematics();

    /*!
     * Destructor
     */
    ~InverseKinematics();

    /*!
     * @brief Loads the kinematic model from an external file.
     *
     * You can specify an optional list specifying which joints
     * are considered as optimization variables (all the joints not
     * contained in the list are considered fixed joint). If the vector is
     * empty all the joints in the model will be considered as optimization variables.
     *
     * @param[in] urdfFile path to the urdf file describing the model
     * @param[in] consideredJoints list of internal joints describing which joints are optimized
     * @param[in] filetype (optional) explicit definition of the type of the loaded file. Only "urdf" is supported at the moment.
     * @return true if successful. False otherwise
     */
    bool loadModelFromFile(const std::string & filename,
                           const std::vector<std::string> &consideredJoints = std::vector<std::string>(),
                           const std::string & filetype="urdf");

    /*!
     * @brief set the kinematic model to be used in the optimization
     *
     * All the degrees of freedom of the model will be used as 
     * optimization variables. If you want to perform the inverse kinematics 
	 * just on a subset of the internal joints of the robot, please use the  
	 * loadReducedModelFromFullModel method contained in the ModelLoader class.
     *
     * @param model the kinematic model to be used in the optimization
     * @return true if successful. False otherwise
     */
    bool setModel(const iDynTree::Model &model);

    /*!
     * Reset the variables.
     * @note the model is not removed
     */
    void clearProblem();

    bool setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName);

    /*!
     * Sets the robot configuration
     *
     *
     * @param baseConfiguration  transformation identifying the base pose with respect to the world frame
     * @param robotConfiguration the robot configuration
     *
     * @return true if successful, false otherwise.
     */
    bool setRobotConfiguration(const iDynTree::Transform& baseConfiguration,
                               const iDynTree::VectorDynSize& jointConfiguration);

    /*!
     * Set configuration for the specified joint
     *
     * @param jointName          name of the joint
     * @param jointConfiguration new value for the joint
     *
     * @return true if successful, false otherwise.
     */
    bool setJointConfiguration(const std::string& jointName,
                               const double jointConfiguration);

    void setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization);

    enum InverseKinematicsRotationParametrization rotationParametrization();

    /*! @name Constraints-related methods
     */
    ///@{

    /*!
     * Adds a (constancy) constraint for the specified frame
     *
     * The constraint is
     * \f$ {}^w_X_{frame}(q) = {}^w_X_{frame}(q^0) \f$
     * where the robot configuration \f$q\f$ is the one specified with setRobotConfiguration
     * @note you should specify first the robot configuration. Otherwise call the versions
     * with explicit constraint value
     * @param frameName the frame name
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameConstraint(const std::string& frameName);

    /*!
     * Adds a (constancy) constraint for the specified frame
     *
     * The homogeneous trasformation of the specified frame w.r.t. the inertial frame
     * will remain constant and equal to the specified second parameter
     *
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the transform to associate to the constraint.
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameConstraint(const std::string& frameName,
                            const iDynTree::Transform& constraintValue);

    /*!
     * Adds a (constancy) position constraint for the specified frame
     *
     * Only the position component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the position associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFramePositionConstraint(const std::string& frameName,
                                    const iDynTree::Position& constraintValue);

    /*!
     * Adds a (constancy) position constraint for the specified frame
     *
     * Only the position component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the position associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFramePositionConstraint(const std::string& frameName,
                                    const iDynTree::Transform& constraintValue);

    /*!
     * Adds a (constancy) orientation constraint for the specified frame
     *
     * Only the orientation component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the orientation associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameRotationConstraint(const std::string& frameName,
                                    const iDynTree::Rotation& constraintValue);

    /*!
     * Adds a (constancy) orientation constraint for the specified frame
     *
     * Only the orientation component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the orientation associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameRotationConstraint(const std::string& frameName,
                                    const iDynTree::Transform& constraintValue);

    ///@}

    /*! @name Target-related methods
     */
    ///@{

    /*!
     * Adds a target for the specified frame
     *
     * @param frameName       the name of the frame which represents the target
     * @param constraintValue value that the frame should reach
     *
     * @return true if successful, false otherwise.
     */
    bool addTarget(const std::string& frameName,
                   const iDynTree::Transform& constraintValue);

    /*!
     * Adds a position (3D) target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param constraintValue value that the origin of the frame frameName should reach
     * @return true if successful, false otherwise.
     */
    bool addPositionTarget(const std::string& frameName,
                           const iDynTree::Position& constraintValue);

    /*!
     * Adds a position (3D) target for the specified frame
     *
     * \note only the position component of the constraintValue parameter
     * will be considered
     * @param frameName the name of the frame which represents the target
     * @param constraintValue value that the origin of the frame frameName should reach
     * @return true if successful, false otherwise.
     */
    bool addPositionTarget(const std::string& frameName,
                           const iDynTree::Transform& constraintValue);

    /*!
     * Adds an orientation target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param constraintValue value that the orientation of the frame frameName should reach
     * @return true if successful, false otherwise.
     */
    bool addRotationTarget(const std::string& frameName,
                           const iDynTree::Rotation& constraintValue);

    /*!
     * Adds an orientation target for the specified frame
     *
     * \note only the orientation component of the constraintValue parameter
     * will be considered
     *
     * This call is equivalent to call
     * @code
     * addRotationTarget(frameName, constraintValue.rotation());
     * @endcode
     * @see 
     * addRotationTarget(const std::string&, const iDynTree::Rotation&)
     * addTarget(const std::string&, const iDynTree::Transform&)
     *
     * @param frameName the name of the frame which represents the target
     * @param constraintValue value that the orientation of the frame frameName should reach
     * @return true if successful, false otherwise.
     */
    bool addRotationTarget(const std::string& frameName,
                           const iDynTree::Transform& constraintValue);


    /*!
     * Specify the method to solve the specified targets
     * 
     * Targets can be solved fully as cost, partially (position or orientation) 
     * as cost and the other component as hard constraint or 
     * fully as hard constraints
     * @see targetResolutionMode()
     *
     * @param mode the target resolution mode
     */
    void setTargetResolutionMode(enum InverseKinematicsTreatTargetAsConstraint mode);

    /*!
     * Return the current target resolution mode
     *
     * @see setTargetResolutionMode
     * @return the current target resolution mode
     */
    enum InverseKinematicsTreatTargetAsConstraint targetResolutionMode();
    ///@}

    /*!
     * Sets a desired final configuration for the joints.
     *
     * The solver will try to obtain solutions as similar to the specified configuration as possible
     *
     * @param desiredJointConfiguration configuration for the joints
     *
     * @return true if successful, false otherwise.
     */
    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    /*!
     * Initial guess for the solution
     *
     * @param baseTransform     initial base pose
     * @param initialCondition  initial joints configuration
     * @return
     */
    bool setInitialCondition(const iDynTree::Transform* baseTransform,
                             const iDynTree::VectorDynSize* initialCondition);



    // This is one part should be checked so as to properly enable warm start
    bool solve();

    /*! @name Solution-related methods
      */
    ///@{

    /*!
     * Initial guess for the solution
     *
     * @param[out] baseTransformSolution  solution for the base position
     * @param[out] shapeSolution       solution for the shape (the internal configurations)
     */
    void getSolution(iDynTree::Transform & baseTransformSolution,
                     iDynTree::VectorDynSize & shapeSolution);

    ///@}


    bool getPoseForFrame(const std::string& frameName, iDynTree::Transform& transform);

    /*
     Other iKin features:
     - set joint limits for each joint (different from the one loaded)
     - block joint
     - Select different solutions for the target
     */

    /* other todos:
     - add check on modelLoaded, and other stuff if needed
     */

    /*!
     *  Access the model used by the InverseKinematics .
     *
     * @return A constant reference to iDynTree::Model used by the inverse kinematics.
     */
    const Model & model() const;

private:
    void* m_pimpl; /*!< private implementation */

};

#endif /* end of include guard: IDYNTREE_INVERSEKINEMATICS_H */
