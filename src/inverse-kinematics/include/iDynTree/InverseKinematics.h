/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef IDYNTREE_INVERSEKINEMATICS_H
#define IDYNTREE_INVERSEKINEMATICS_H

#include <string>
#include <vector>

#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/Core/Direction.h>

namespace iDynTree {
    class VectorDynSize;
    class Transform;
    class Position;
    class Rotation;
    class Model;
    class Polygon;
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

    inline int sizeOfRotationParametrization(enum InverseKinematicsRotationParametrization rotationParametrization)
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

/*!
 * \ingroup iDynTreeExperimental
 *
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
 * @warning This class is still in active development, and so API interface can change between iDynTree versions.
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
     * All the degrees of freedom listed in the second parameters will be used as
     * optimization variables. 
     * If the vector is empty, all the joints will be used.
     *
     * @note you may want to simplify the model by calling
	 * loadReducedModelFromFullModel method contained in the ModelLoader class.
     *
     * @param model the kinematic model to be used in the optimization
     * @return true if successful. False otherwise
     */
    bool setModel(const iDynTree::Model &model,
                  const std::vector<std::string> &consideredJoints = std::vector<std::string>());
    
    /*!
     * Set new joint limits
     * \author Yue Hu
     * @param jointLimits vector of new joint limits to be imposed
     * @return true if successfull, false otherwise
     */
    bool setJointLimits(std::vector<std::pair<double, double> >& jointLimits);
    
    /*!
     * Get current joint limits
     * \author Yue Hu
     * @param jointLimits vector of current joint limits
     * @return true if successfull, false otherwise
     */
    bool getJointLimits(std::vector<std::pair<double, double> >& jointLimits);

    /*!
     * Reset the variables.
     * @note the model is not removed
     * @note The parameters such as max iterations, max cpu time and verbosity are resetted with this method.
     */
    void clearProblem();

    bool setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName);

    /*!
     * Sets the robot current configuration
     *
     *
     * @param baseConfiguration  transformation identifying the base pose with respect to the world frame
     * @param robotConfiguration the robot configuration
     *
     * @note the size (and order) of jointConfiguration must match the joints in the model, not
     * in the consideredJoints variable
     *
     * @return true if successful, false otherwise.
     */
    bool setCurrentRobotConfiguration(const iDynTree::Transform& baseConfiguration,
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

    /*! @name Parameters-related methods
     */
    ///@{

    /**
     * Sets Maximum Iteration.
     *
     * The default value for this parameter is 3000 .
     *
     * @param max_iter exits if iter>=max_iter (max_iter<0
     *                   disables this check).
     */
    void setMaxIterations(const int max_iter);

    /**
     * Retrieves the current value of Maximum Iteration.
     * @return max_iter.
     */
    int maxIterations() const;

    /**
     * Sets Maximum CPU seconds.
     *
     * The default value for this parameter is \frac$ 10^{6} \frac$ .
     *
     * @param max_cpu_time exits if cpu_time>=max_cpu_time given in
     *                     seconds.
     */
    void setMaxCPUTime(const double max_cpu_time);

    /**
     * Retrieves the current value of Maximum CPU seconds.
     * @return max_cpu_time.
     */
    double maxCPUTime() const;

    /**
     * Sets cost function tolerance.
     *
     * The default value for this parameter is \frac$ 10^{-8} \frac$  .
     *
     * @param tol tolerance.
     */
    void setCostTolerance(const double tol);

    /**
     * Retrieves cost function tolerance.
     * @return tolerance.
     */
    double costTolerance() const;

    /**
     * Sets constraints tolerance.
     *
     * The default value for this parameter is \frac$ 10^{-4} \frac$  .
     *
     * @param tol tolerance.
     */
    void setConstraintsTolerance(const double constr_tol);

    /**
     * Retrieves constraints tolerance.
     * @return tolerance.
     */
    double constraintsTolerance() const;

    /*!
     * Sets Verbosity.
     * @param verbose is a integer number which progressively enables
     *                different levels of warning messages or status
     *                dump. The larger this value the more detailed
     *                is the output.
    */
    void setVerbosity(const unsigned int verbose);

    std::string linearSolverName();
    void setLinearSolverName(const std::string &solverName);

    ///@}


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

    /*!
     * Activate a given constraint previously added with an addFrame**Constraint method.
     *
     * \note In this version of iDynTree, it is not possible to change the nature of the constraint
     *       (Full, Position or Rotation) when activating it again.
     *
     * @note This method returns true even if the frame constraint was already activate, it only
     *       returns false if the constraint was never added.
     * @warning This method is not meant to be called at each IK loop, and it can increase the computational
     *          time of the next call to solve.
     *
     * @param frameName       the name of the frame on which to attach the constraint
     * @param newConstraintValue the pose of the constrained frame (r) in the world frame (w), i.e. ʷHᵣ .
     * @return true if successful, false otherwise.
     */
    bool activateFrameConstraint(const std::string& frameName,
                                 const Transform& newConstraintValue);
    /*!
     * Deactivate a given constraint previously added with an addFrame**Constraint method.
     *
     * @note This method returns true even if the frame constraint was already deactivated, it only
     *       returns false if the constraint was never added.
     *
     * @param frameName       the name of the frame on which to attach the constraint
     * @return true if successful (i.e. the constraint is present) , false otherwise.
     */
    bool deactivateFrameConstraint(const std::string& frameName);

    /*!
     * Check if a given constraint is active or not.

     *
     * @param frameName       the name of the constrained frame
     * @return true if the constraint is active, false if it is not active or it does not exist, or if the frame does not exist.
     */
    bool isFrameConstraintActive(const std::string& frameName) const;

    /*!
     * Specialization of addCenterOfMassProjectionConstraint when only two support frames are specified.
     */
    bool addCenterOfMassProjectionConstraint(const std::string& firstSupportFrame,
                                             const Polygon& firstSupportPolygon,
                                             const iDynTree::Direction xAxisOfPlaneInWorld,
                                             const iDynTree::Direction yAxisOfPlaneInWorld,
                                             const iDynTree::Position originOfPlaneInWorld = iDynTree::Position::Zero());

    /*!
     * Specialization of addCenterOfMassProjectionConstraint when only two support frames are specified.
     */
    bool addCenterOfMassProjectionConstraint(const std::string& firstSupportFrame,
                                             const Polygon& firstSupportPolygon,
                                             const std::string& secondSupportFrame,
                                             const Polygon& secondSupportPolygon,
                                             const iDynTree::Direction xAxisOfPlaneInWorld,
                                             const iDynTree::Direction yAxisOfPlaneInWorld,
                                             const iDynTree::Position originOfPlaneInWorld = iDynTree::Position::Zero());

    /*!
     * Add a constant inequality constraint on the projection of the center of mass,
     * assuming an arbitrary number of support links.
     *
     * If a subset of the supportFrames is contrained by a FrameConstraint (both position and constraint) and such
     * constraint is active, this constraint adds a inequality constraint to ensure that the center of mass projection
     * lies on the convex hull of the contact polygons.
     */
    bool addCenterOfMassProjectionConstraint(const std::vector<std::string>& supportFrames,
                                             const std::vector<Polygon>& supportPolygons,
                                             const iDynTree::Direction xAxisOfPlaneInWorld,
                                             const iDynTree::Direction yAxisOfPlaneInWorld,
                                             const iDynTree::Position originOfPlaneInWorld);

    /*!
     * Get the distance between the projection of the center of mass projection for the current configuration (set through setRobotConfiguration)
     * and the limit of the convex hull (positive if the center of mass is inside the convex hull, negative if the com is outside the convex hull).
     *
     * If no constraint has been added through a call to addCenterOfMassProjectionConstraint, return 0.0 .
     *
     * \todo Move this function in a contraint-specific class.
     */
    double getCenterOfMassProjectionMargin();


    /*!
     *
     * \brief Get the active convex hull
     *
     * The convex hull returned is expressed in the plane defined by the xAxisOfPlaneInWorld,
     * yAxisOfPlaneInWorld and originOfPlaneInWorld arguments of the addCenterOfMassProjectionConstraint methods.
     *
     * The transform world_H_constraintFrame that describe how the support polygon for each support frame is transformed
     * in the world frame are the one set in the addFrameConstraint method.
     *
     * @param[out] convexHull constraint convex hull for the projected center of mass.
     * @return true if the center of mass projection constraint is active, false otherwise.
     */
    bool getCenterOfMassProjectConstraintConvexHull(Polygon2D& convexHull);

    ///@}

    /*! @name Target-related methods
     */
    ///@{

    /*!
     * Adds a target for the specified frame
     *
     * @param frameName       the name of the frame which represents the target
     * @param targetValue value that the frame should reach
     * @param[in] positionWeight if the position part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @param[in] rotationWeight if the rotation part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @return true if successful, false otherwise.
     */
    bool addTarget(const std::string& frameName,
                   const iDynTree::Transform& targetValue,
                   const double positionWeight=1.0,
                   const double rotationWeight=1.0);

    /*!
     * Adds a position (3D) target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the origin of the frame frameName should reach
     * @param[in] positionWeight if the position part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @return true if successful, false otherwise.
     */
    bool addPositionTarget(const std::string& frameName,
                           const iDynTree::Position& targetValue,
                           const double positionWeight=1.0);

    /*!
     * Adds a position (3D) target for the specified frame
     *
     * \note only the position component of the targetValue parameter
     * will be considered
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the origin of the frame frameName should reach
     * @param[in] positionWeight if the position part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @return true if successful, false otherwise.
     */
    bool addPositionTarget(const std::string& frameName,
                           const iDynTree::Transform& targetValue,
                           const double positionWeight=1.0);

    /*!
     * Adds an orientation target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the orientation of the frame frameName should reach
     * @param[in] rotationWeight if the rotation part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @return true if successful, false otherwise.
     */
    bool addRotationTarget(const std::string& frameName,
                           const iDynTree::Rotation& targetValue,
                           const double rotationWeight=1.0);

    /*!
     * Adds an orientation target for the specified frame
     *
     * \note only the orientation component of the targetValue parameter
     * will be considered
     *
     * This call is equivalent to call
     * @code
     * addRotationTarget(frameName, targetValue.rotation());
     * @endcode
     * @see
     * addRotationTarget(const std::string&, const iDynTree::Rotation&)
     * addTarget(const std::string&, const iDynTree::Transform&)
     *
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the orientation of the frame frameName should reach
     * @param[in] rotationWeight if the rotation part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is 1.0
     * @return true if successful, false otherwise.
     */
    bool addRotationTarget(const std::string& frameName,
                           const iDynTree::Transform& targetValue,
                           const double rotationWeight=1.0);

    /*!
     * Update the desired target and weights for the specified frame.
     *
     * @param frameName       the name of the frame which represents the target
     * @param targetValue value that the frame should reach
     * @param[in] positionWeight if the position part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is the the last one previously set.
     * @param[in] rotationWeight if the rotation part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is the the last one previously set.
     * @return true if successful, false otherwise, for example if the specified frame target was not previously added with addTarget .
     */
    bool updateTarget(const std::string& frameName,
                      const iDynTree::Transform& targetValue,
                      const double positionWeight=-1.0,
                      const double rotationWeight=-1.0);

    /*!
     * Update the position (3D) target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the origin of the frame frameName should reach
     * @param[in] positionWeight if the position part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is the last one previously set.
     * @return true if successful, false otherwise, for example if the specified frame target was not previously added with addTarget or addPositionTarget .
     */
    bool updatePositionTarget(const std::string& frameName,
                              const iDynTree::Position& targetValue,
                              const double positionWeight=-1.0);
    /*!
     * Update an orientation target for the specified frame
     *
     * @param frameName the name of the frame which represents the target
     * @param targetValue value that the orientation of the frame frameName should reach
     * @param[in] rotationWeight if the rotation part of the target is handled as
     *                           a term in the cost function, this specify the weight
     *                           of this term in the cost function. Default value is the last one previously set.
     * @return true if successful, false otherwise, for example if the specified frame target was not previously added with addTarget or addRotationTarget .
     */
    bool updateRotationTarget(const std::string& frameName,
                              const iDynTree::Rotation& targetValue,
                              const double rotationWeight=-1.0);


    /*!
     * Specify the default method to solve all the specified targets
     *
     * Targets can be solved fully as cost, partially (position or orientation)
     * as cost and the other component as hard constraint or
     * fully as hard constraints
     * @see targetResolutionMode()
     * All the newly added target will have as default resolution mode the one specified in this
     * function. Existing targets will remain unchanged
     *
     * @param mode the target resolution mode
     * 
     * @return true if successful, false otherwise.
     */
    void setDefaultTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode);

    /*!
     * Returns the default method to solve all the specified targets
     *
     * @see setDefaultTargetResolutionMode()
     * All the newly added target will have as default resolution mode the one returned in this
     * function.
     *
     * @return the current default method to solve targets
     */
    enum iDynTree::InverseKinematicsTreatTargetAsConstraint defaultTargetResolutionMode();

    /*!
     * Specify the method to solve the specified target
     *
     * Targets can be solved fully as cost, partially (position or orientation)
     * as cost and the other component as hard constraint or
     * fully as hard constraints
     * @see targetResolutionMode()
     *
     * @param mode the target resolution mode
     * @param targetName the name (frame) identified the target
     * @return true if successful, false otherwise, for example if the specified frame target was not previously added with addTarget or addRotationTarget .
     */
    bool setTargetResolutionMode(const std::string& targetName, enum InverseKinematicsTreatTargetAsConstraint mode);
    

    /*!
     * Return the target resolution mode
     *
     * @see setTargetResolutionMode
     * @note If the target hasn't been specified yet, it returns "InverseKinematicsTreatTargetAsConstraintNone"
     * @return the current target resolution mode, or InverseKinematicsTreatTargetAsConstraintNone if the target cannot be found
     */
    enum InverseKinematicsTreatTargetAsConstraint targetResolutionMode(const std::string& frameName);
    ///@}

    /*!
     * Sets a desired final configuration for all the robot joints.
     *
     * The solver will try to obtain solutions as similar to the specified configuration as possible
     *
     * @note the desiredJointConfiguration have the same serialisation of the joints in the specified model
     *
     * @param[in] desiredJointConfiguration configuration for the joints
     * @param[in] weight weight for the joint configuration cost.
     *                   If it is not passed, the previous passed value will be mantained.
     *                   If the value was never passed, its value is 1e-6 .
     *
     * @return true if successful, false otherwise.
     */
    bool setDesiredFullJointsConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, double weight=-1.0);

    /*!
     * Sets a desired final configuration for all the robot joints.
     *
     * The solver will try to obtain solutions as similar to the specified configuration as possible
     *
     * @note the desiredJointConfiguration have the same serialisation of the joints in the specified model
     *
     * @param[in] desiredJointConfiguration configuration for the joints
     * @param[in] weights Joint-wise weights for the joint configuration cost.
     *                   This vector should have the same dimension of the desiredJointConfiguration.
     *                   If one of its elements is negative, the previous value will be kept.
     *                   If the value was never passed, its value is 1e-6, equal for all joints.
     *
     * @return true if successful, false otherwise.
     */
    bool setDesiredFullJointsConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, const iDynTree::VectorDynSize& weights);

    /*!
     * Sets a desired final configuration for the set of considered joints.
     *
     * The solver will try to obtain solutions as similar to the specified configuration as possible
     *
     * @note the desiredJointConfiguration have the same order of the joints in the consideredJoints list.
     *
     * @param[in] desiredJointConfiguration configuration for the joints
     * @param[in] weight weight for the joint configuration cost.
     *                   If it is not passed, the previous passed value will be mantained.
     *                   If the value was never passed, its value is 1e-6 .
     *
     * @return true if successful, false otherwise.
     */
    bool setDesiredReducedJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, double weight=-1.0);

    /*!
     * Sets a desired final configuration for the set of considered joints.
     *
     * The solver will try to obtain solutions as similar to the specified configuration as possible
     *
     * @note the desiredJointConfiguration have the same order of the joints in the consideredJoints list.
     *
     * @param[in] desiredJointConfiguration configuration for the joints
     * @param[in] weights Joint-wise weights for the joint configuration cost.
     *                   This vector should have the same dimension of the desiredJointConfiguration.
     *                   If one of its elements is negative, the previous value will be kept.
     *                   If the value was never passed, its value is 1e-6, equal for all joints.
     *
     * @return true if successful, false otherwise.
     */
    bool setDesiredReducedJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, const iDynTree::VectorDynSize& weights);

    bool setFullJointsInitialCondition(const iDynTree::Transform* baseTransform,
                                       const iDynTree::VectorDynSize* initialCondition);
    bool setReducedInitialCondition(const iDynTree::Transform* baseTransform,
                                    const iDynTree::VectorDynSize* initialCondition);

    // This is one part should be checked so as to properly enable warm start
    bool solve();

    /*! @name Solution-related methods
      */
    ///@{

    void getFullJointsSolution(iDynTree::Transform& baseTransformSolution,
                               iDynTree::VectorDynSize& shapeSolution);

    /*!
     * Return the last solution of the inverse kinematics problem
     *
     * This method returns in the shapeSolution variable only the joints that have been optimised
     * viz. only the joints specified in the consideredJoints variable in the initialization 
     * @param[out] baseTransformSolution  solution for the base position
     * @param[out] shapeSolution       solution for the shape (the internal configurations)
     */
    void getReducedSolution(iDynTree::Transform& baseTransformSolution,
                            iDynTree::VectorDynSize& shapeSolution);

    ///@}


    bool getPoseForFrame(const std::string& frameName, iDynTree::Transform& transform);

    /*
     Other iKin features:
     - set joint limits for each joint (different from the one loaded)
     - Select different solutions for the target
     */

    /* other todos:
     - add check on modelLoaded, and other stuff if needed
     */

    const Model & fullModel() const;

    const Model & reducedModel() const;

    void setCOMTarget(iDynTree::Position& desiredPosition, double weight = 1.0);

    void setCOMAsConstraint(bool asConstraint = true);

    void setCOMAsConstraintTolerance(double tolerance = 1e-8);

    bool isCOMAConstraint();

    bool isCOMTargetActive();

    void deactivateCOMTarget();

    /*!
     * Set the directions along which a point will be projected.
     *
     * \author Aiko Dinale (29/08/2017)
     *
     * @param direction    vector along which we want to project a point
     */
    void setCOMConstraintProjectionDirection(iDynTree::Vector3 direction);

private:
    void* m_pimpl; /*!< private implementation */

};

#endif /* end of include guard: IDYNTREE_INVERSEKINEMATICS_H */
