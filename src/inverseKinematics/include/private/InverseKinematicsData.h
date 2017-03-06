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

#include "InverseKinematics.h"

namespace iDynTree {
    class Model;
}

namespace internal {
namespace kinematics{

    class InverseKinematicsData;
    class Transform;
    typedef std::map<int, internal::kinematics::Transform> TransformMap; //ordered map. Order is important

    class InverseKinematicsNLP;
}
}

class internal::kinematics::InverseKinematicsData {

    //forbid copy
    InverseKinematicsData(const InverseKinematicsData&);
    InverseKinematicsData& operator=(const InverseKinematicsData&);

    //!!!: I have to divide variables between the optimized one (buffers inside the Solver, except results and I/O variables here)
    // and the "model" variables.


    /*! @name Model-related variables
     */
    ///@{
    iDynTree::KinDynComputations m_dynamics; /*!< object for kinematics and dynamics computation */

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

    std::vector<std::pair<double, double> > m_jointLimits; /*!< Limits for joints. The pair is ordered as min and max */

    ///@}

    /*! @name Optimization-related variables
     */
    ///@{

    enum iDynTree::InverseKinematicsRotationParametrization m_rotationParametrization; /*!< type of parametrization of the orientation */

    TransformMap m_constraints; /*!< list of hard constraints */
    TransformMap m_targets; /*!< list of targets */

    //Preferred joints configuration for the optimization
    //Size #size of optimization variables
    iDynTree::VectorDynSize m_preferredJointsConfiguration;

    bool m_areInitialConditionsSet; /*!< True if initial condition are provided by the user */
    enum iDynTree::InverseKinematicsTreatTargetAsConstraint m_targetResolutionMode; /*!< Specify how targets are solved (Partially/Fully in cost or as hard constraints) */

    //Result of optimization
    //These variables also containts the initial condition if
    //the solver has not obtained a solution yet
    //Actually they contains either the initial condition or the solution
    iDynTree::VectorDynSize m_optimizedRobotDofs;
    iDynTree::Position m_optimizedBasePosition;
    iDynTree::Vector4 m_optimizedBaseOrientation;

    ///@}

    void updateRobotConfiguration();
public:
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
    bool addFrameConstraint(const internal::kinematics::Transform& frameTransform);


    /*!
     * Add a target for the specified frame
     *
     * @param frameTransform the frame to be considered as a target
     * @param weight weight for the associated target. Currently ignored
     * @return true if successfull, false otherwise
     */
    bool addTarget(const internal::kinematics::Transform& frameTransform, double weight = 1);


    /*!
     * Set the initial robot configuration
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
     * Set the desired robot joint configurations, i.e. guess for the solver
     *
     * @param desiredJointConfiguration desired joint configuration
     * @return true if successfull, false otherwise
     */
    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);


    /*!

     I don't remember
     @param baseTransform
     @param initialCondition
     @return true if successfull, false otherwise
     */
    bool setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition);

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


    /*!
     * Prepare the internal data to run an optimization
     */
    void prepareForOptimization();

    friend class InverseKinematicsNLP;


    /*!
     * Access the Kinematics and Dynamics object used by the solver
     *
     * @return reference to the kinematics and dynamics object
     */
    iDynTree::KinDynComputations& dynamics();

    Ipopt::SmartPtr<Ipopt::IpoptApplication> solver; /*!< I don't remember why I put it public and not private */

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H */
