/*!
 * @file InverseKinematicsData.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
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
    enum iDynTree::InverseKinematicsTreatTargetAsConstraint targetResolutionMode; /*!< Specify how targets are solved (Partially/Fully in cost or as hard constraints) */

    //Result of optimization
    //These variables also containts the initial condition if
    //the solver has not obtained a solution yet
    iDynTree::VectorDynSize m_optimizedRobotDofs;
    iDynTree::Position m_optimizedBasePosition;
    iDynTree::Vector4 m_optimizedBaseOrientation;

    ///@}

    void updateRobotConfiguration();
public:
    InverseKinematicsData();

    bool setModel(const iDynTree::Model& model);

    /**
     * Reset variables to defaults
     *
     * If the model has been loaded, defaults means #dofs size
     * Otherwise the size is zero.
     * All constraints are reset.
     */
    void clearProblem();

    bool addFrameConstraint(const internal::kinematics::Transform& frameTransform);
    bool addTarget(const internal::kinematics::Transform& frameTransform, double weight = 1);

    bool setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration);
    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    
    bool setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration);
    bool setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition);

    void setRotationParametrization(enum iDynTree::InverseKinematicsRotationParametrization parametrization);
    enum iDynTree::InverseKinematicsRotationParametrization rotationParametrization();

    void setTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode);

    void prepareForOptimization();

    friend class InverseKinematicsNLP;


    iDynTree::KinDynComputations& dynamics();
    Ipopt::SmartPtr<Ipopt::IpoptApplication> solver;

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSDATA_H */
