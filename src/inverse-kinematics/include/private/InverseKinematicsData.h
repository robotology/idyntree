/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H
#define IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H

#include "InverseKinematicsNLP.h"
#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/InverseKinematics.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

#include <IpIpoptApplication.hpp>


#include <vector>
#include <map>
#include <unordered_map>

namespace internal {
namespace kinematics{

    class InverseKinematicsData;
    class TransformConstraint;
    typedef std::map<int, internal::kinematics::TransformConstraint> TransformMap; //ordered map. Order is important

    class InverseKinematicsNLP;
}
}

class internal::kinematics::InverseKinematicsData {
    //Declare as friend the IKNLP class so as it can access the private data
    friend class InverseKinematicsNLP;
    // and also inverseKineamtics
    friend class iDynTree::InverseKinematics;

    //forbid copy
    InverseKinematicsData(const InverseKinematicsData&);
    InverseKinematicsData& operator=(const InverseKinematicsData&);

    struct {
        bool isActive;
        iDynTree::Position desiredPosition;
        double weight;
        bool isConstraint;
        double constraintTolerance;
    } m_comTarget;

    // The variables are divided among the optimized one (buffers inside the Solver, except results and I/O variables here),
    // the "model" variables and the parameters of the optimization.

    iDynTree::InverseKinematicsTreatTargetAsConstraint m_defaultTargetResolutionMode;

    enum InverseKinematicsInitialConditionType {
        InverseKinematicsInitialConditionNotSet,
        InverseKinematicsInitialConditionPartial,
        InverseKinematicsInitialConditionFull
    };

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

    size_t m_dofs; /*!< internal DoFs of the model, i.e. size of joint vectors */
    struct {
        std::vector<bool> fixedVariables; /* for each variable it says if it is fixed or optimisation variable */
        std::unordered_map<int, int> modelJointsToOptimisedJoints; // that is key = index in the reduced set of variables, value = index in the full model
        iDynTree::Model reducedModel;
    } m_reducedVariablesInfo;

    ///@}

    /*! @name Optimization-related variables
     */
    ///@{

    enum iDynTree::InverseKinematicsRotationParametrization m_rotationParametrization; /*!< type of parametrization of the orientation */

    TransformMap m_constraints; /*!< list of hard constraints */
    TransformMap m_targets; /*!< list of targets */

    // Attributes relative to the COM Projection constraint (TODO(traversaro): move most of them in a constraint-specific class)
    iDynTree::ConvexHullProjectionConstraint m_comHullConstraint; /*!< Helper to implement COM constraint */
    iDynTree::Vector3 m_comHullConstraint_projDirection;
    std::vector<iDynTree::FrameIndex> m_comHullConstraint_supportFramesIndeces;
    std::vector<iDynTree::Polygon> m_comHullConstraint_supportPolygons;
    iDynTree::Direction m_comHullConstraint_xAxisOfPlaneInWorld;
    iDynTree::Direction m_comHullConstraint_yAxisOfPlaneInWorld;
    iDynTree::Position m_comHullConstraint_originOfPlaneInWorld;

    //Preferred joints configuration for the optimization
    //Size: getNrOfDOFs of the considered model
    iDynTree::VectorDynSize m_preferredJointsConfiguration;
    iDynTree::VectorDynSize m_preferredJointsWeight;

    bool m_areBaseInitialConditionsSet; /*!< True if initial condition for the base pose are provided by the user */

    InverseKinematicsInitialConditionType m_areJointsInitialConditionsSet; /*!< specify if the initial condition for the joints are provided by the user */

    //These variables containts the initial condition
    iDynTree::Transform m_baseInitialCondition;
    iDynTree::VectorDynSize m_jointInitialConditions;

    //Result of optimization
    iDynTree::Transform m_baseResults;
    iDynTree::VectorDynSize m_jointsResults;
    iDynTree::VectorDynSize m_constraintMultipliers;
    iDynTree::VectorDynSize m_lowerBoundMultipliers;
    iDynTree::VectorDynSize m_upperBoundMultipliers;

    ///@}

    bool m_problemInitialized;
    bool m_warmStartEnabled;
    size_t m_numberOfOptimisationVariables;
    size_t m_numberOfOptimisationConstraints;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> m_solver; /*!< Instance of IPOPT solver */
    Ipopt::SmartPtr<internal::kinematics::InverseKinematicsNLP> m_nlpProblem;

    /*!
     * Update internal variables given a change in the robot state
     */
    void updateRobotConfiguration();

    /*!
     * Prepare the internal data to run an optimization
     */
    void prepareForOptimization();

    /*!
     * compute the problem size (number of optimisation variables and constraints)
     */
    void computeProblemSizeAndResizeBuffers();

    /*!
     * Configure the COM projection constraints given the current active contraints.
     */
    void configureCenterOfMassProjectionConstraint();

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
    bool setModel(const iDynTree::Model& model, const std::vector<std::string> &consideredJoints = std::vector<std::string>());

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
     * i.e. as soft or hard constraints. It applies only to the newly added targets
     *
     * @param mode how to treat the targets
     */
    void setDefaultTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode);

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint defaultTargetResolutionMode();

    /*!
     * Set how the specified target should be considered in the optimization problem
     * i.e. as soft or hard constraints
     *
     * @param mode how to treat the target
     */
    void setTargetResolutionMode(TransformMap::iterator target, enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode);

    /*! Return the resolution mode adopted for the specified target
     * @return the resolution mode
     */
    enum iDynTree::InverseKinematicsTreatTargetAsConstraint targetResolutionMode(TransformMap::iterator target) const;

    /*! Solve the NLP problem
     *
     * @return true if the problem is solved. False otherwise
     */
    bool solveProblem();

    /*!
     * Access the Kinematics and Dynamics object used by the solver
     *
     * @return reference to the kinematics and dynamics object
     */
    iDynTree::KinDynComputations& dynamics();

    void setCoMTarget(const iDynTree::Position& desiredPosition, double weight);

    void setCoMasConstraint(bool asConstraint);

    bool isCoMaConstraint();

    void setCoMasConstraintTolerance(double TOL);

    bool isCoMTargetActive();

    void setCoMTargetInactive();

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H */
