/*!
 * @file InverseKinematicsData.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#ifndef IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H
#define IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <vector>
#include <map>
#include <IpIpoptApplication.hpp>

#include <iDynTree/InverseKinematics.h>

namespace iDynTree {
    class Model;
}

namespace internal {
namespace kinematics{

    class InverseKinematicsData;
    class TransformConstraint;
    typedef std::map<int, internal::kinematics::TransformConstraint> TransformMap; //ordered map. Order is important

    class InverseKinematicsNLP;
}
}

class internal::kinematics::InverseKinematicsData {

    //forbid copy
    InverseKinematicsData(const InverseKinematicsData&);
    InverseKinematicsData& operator=(const InverseKinematicsData&);

    // The variables are divided among the optimized one (buffers inside the Solver, except results and I/O variables here),
    // the "model" variables and the parameters of the optimization.

public:
    /*! @name Model-related variables
     */
    ///@{
    iDynTree::KinDynComputations m_dynamics; /*!< object for kinematics and dynamics computation */

    std::vector<std::pair<double, double> > m_jointLimits; /*!< Limits for joints. The pair is ordered as min and max */

    /*!
     * Variables needed to identify the state of the robot
     * i.e. position and velocity
     */
    struct {
        iDynTree::VectorDynSize jointsConfiguration; /*!< joint configuration \f$ q_j \in \mathbb{R}^n \f$ */
        iDynTree::Transform basePose; /*!< base position \f$ w_H_{base} \in SE(3) \f$ */
        iDynTree::VectorDynSize jointsVelocity; /*!< joint velotiy \f$ \dot{q}_j \in \mathbb{R}^n \f$ */
        iDynTree::Twist baseTwist; /*!< base velocity \f$ [\dot{p}, {}^I \omega] \in se(3) \f$ */
        iDynTree::Vector3 worldGravity; /*!< gravity acceleration in inertial frame, i.e. -9.81 along z */
    } m_state;

    unsigned m_dofs; /*!< internal DoFs of the model, i.e. size of joint vectors */

    ///@}

    /*! @name Optimization-related variables
     */
    ///@{

    enum iDynTree::InverseKinematicsRotationParametrization m_rotationParametrization; /*!< type of parametrization of the orientation */

    TransformMap m_constraints; /*!< list of hard constraints */
    TransformMap m_targets; /*!< list of targets */

    //Preferred joints configuration for the optimization
    //Size: getNrOfDOFs of the considered model
    iDynTree::VectorDynSize m_preferredJointsConfiguration;
    double m_preferredJointsWeight;

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint m_targetResolutionMode; /*!< Specify how targets are solved (Partially/Fully in cost or as hard constraints) */

    bool m_areBaseInitialConditionsSet; /*!< True if initial condition for the base pose are provided by the user */
    bool m_areJointsInitialConditionsSet; /*!< True if initial condition for the joints are provided by the user */

    //These variables containts the initial condition
    iDynTree::Transform m_baseInitialCondition;
    iDynTree::VectorDynSize m_jointInitialConditions;

    //Result of optimization
    iDynTree::Transform m_baseResults;
    iDynTree::VectorDynSize m_jointsResults;

    ///@}

    Ipopt::SmartPtr<Ipopt::IpoptApplication> m_solver; /*!< Instance of IPOPT solver */

    /*!
     * Update internal variables given a change in the robot state
     */
    void updateRobotConfiguration();

    /*!
     * Prepare the internal data to run an optimization
     */
    void prepareForOptimization();

    /*! @name Optimization-related parameters
     */
    ///@{

    int m_maxIter; /*!< Maximum number of iterations */
    double m_maxCpuTime; /*!< Maximum CPU time */
    double m_tol; /*!< Tolerance for the cost */
    double m_constrTol; /*!< Tolerance for the constraints */
    int m_verbosityLevel; /*!< Verbosity level */
    std::string m_solverName;

    ///@}


    /*!
     * Default constructor
     */
    InverseKinematicsData();

    /*!
     * Set the kinematic model to be used for the computations
     * @param model the model to be used
     * @return true if successfull, false otherwise
     */
    bool setModel(const iDynTree::Model& model);

    /*!
     * Reset variables to defaults
     *
     * If the model has been loaded, defaults means #dofs size
     * Otherwise the size is zero.
     * All constraints are reset.
     */
    void clearProblem();

    /*!
     * Add a constraint for the specified frame
     *
     * @param frameTransform the frame to be considered as a constraint
     * @return true if successfull, false otherwise
     */
    bool addFrameConstraint(const internal::kinematics::TransformConstraint& frameTransformConstraint);

    /*!
     * Add a target for the specified frame
     *
     * @param frameTransform the frame to be considered as a target
     * @return true if successfull, false otherwise
     */
    bool addTarget(const internal::kinematics::TransformConstraint& frameTransform);

    /*!
     * Get target if it exists.
     *
     * Get a reference to a target if it exists, or
     * return m_targets::end() print an error otherwise.
     */
    TransformMap::iterator getTargetRefIfItExists(const std::string targetFrameName);

    /*!
     * Update the position reference for a target
     *
     */
    void updatePositionTarget(TransformMap::iterator target, iDynTree::Position newPos, double newPosWeight);

    /*!
     * Update the position reference for a target
     *
     */
    void updateRotationTarget(TransformMap::iterator target, iDynTree::Rotation newRot, double newRotWeight);


    /*!
     * Set the current robot configuration
     *
     * This confguration will be used for all internal calls to kinematics functions
     *
     * @note if setInitialCondition is not called then the robot configuration is assumed
     * as initial condition
     * @see setJointConfiguration
     *
     * @param baseConfiguration base configuration (position and orientation)
     * @param jointConfiguration joints configuration
     * @return true if successfull, false otherwise
     */
    bool setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration);

    /*!
     * Set the initial robot joint configuration for a specified joint
     *
     * @param jointName name of the joint
     * @param jointConfiguration value of the joint
     * @return true if successfull, false otherwise
     */
    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);

    /*!
     * Set the desired robot joint configurations
     *
     * This term will be used as reference for the regularization term
     *
     * @param desiredJointConfiguration desired joint configuration
     * @param desiredJointWeight weight for the regularization term
     * @return true if successfull, false otherwise
     */
    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, const double desiredJointWeight);

    /*!
     * Set the initial condition for the solver (i.e. guess solution)
     *
     *
     * @param baseTransform the guess for the base pose or NULL if no guess is provided
     * @param jointInitialConditions the guess for the joint variables or NULL if no guess is provided
     * @return true if successfull, false otherwise
     */
    bool setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* jointInitialConditions);

    /*!
     * Set the type of parametrization for the SO3 (rotation)
     * Currently RPY and Quaternion are supported
     * @param parametrization type of parametrization
     */
    void setRotationParametrization(enum iDynTree::InverseKinematicsRotationParametrization parametrization);

    /*! Return the current rotation parametrization used by the solver
     * @return the current rotation parametrization
     */
    enum iDynTree::InverseKinematicsRotationParametrization rotationParametrization();

    /*!
     * Set how targets should be considered in the optimization problem
     * i.e. as soft or hard constraints
     *
     * @param mode how to treat the targets
     */
    void setTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode);

    /*! Return the current rotation parametrization used by the solver
     * @return the current rotation parametrization
     */
    enum iDynTree::InverseKinematicsTreatTargetAsConstraint targetResolutionMode();

    /*! Solve the NLP problem
     *
     * @return true if the problem is solved. False otherwise
     */
    bool solveProblem();

    void getSolution(iDynTree::Transform & baseTransformSolution,
                     iDynTree::VectorDynSize & shapeSolution);

    /*!
     * Access the Kinematics and Dynamics object used by the solver
     *
     * @return reference to the kinematics and dynamics object
     */
    iDynTree::KinDynComputations& dynamics();


    //Declare as friend the IKNLP class so as it can access the private data
    friend class InverseKinematicsNLP;

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H */
