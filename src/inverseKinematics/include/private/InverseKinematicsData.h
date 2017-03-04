/*!
 * @file InverseKinematicsData.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H
#define IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H

#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <vector>
#include <map>
#include <IpIpoptApplication.hpp>

#include "InverseKinematics.h"

namespace kinematics {
    class InverseKinematicsData;
    class Transform;
    typedef std::map<int, kinematics::Transform> TransformMap; //ordered map. Order is important

    class InverseKinematicsNLP;
}

class kinematics::InverseKinematicsData {

//    InverseKinematicsData(const InverseKinematicsData&);
//    InverseKinematicsData& operator=(const InverseKinematicsData&);

    //!!!: I have to divide variables between the optimized one (buffers inside the Solver, but results and I/O variables here)
    // and the "model" variables.

    //Model section
    iDynTree::HighLevel::DynamicsComputations m_dynamics;

    //Initial robot state
    struct {
        iDynTree::VectorDynSize jointsConfiguration; //Size dofs
        iDynTree::Transform basePose;
        iDynTree::VectorDynSize jointsVelocityAndAcceleration; //Size dofs - set to zero
        iDynTree::Twist baseTwist;
        iDynTree::ClassicalAcc baseAcceleration;
        iDynTree::SpatialAcc worldGravity;
    } m_state;

    //Number of Dofs in the model
    unsigned m_dofs;

    std::vector<std::pair<double, double> > m_jointLimits; //limits for joints (Dofs)

    //END model section

    //Optimization section

    enum InverseKinematicsRotationParametrization m_rotationParametrization;

    //Joint - variables mapping. By default they match the Dofs
    std::vector<int> m_variablesToJointsMapping;

    //Constraints
    TransformMap m_constraints;
    TransformMap m_targets;

    //Preferred joints configuration for the optimization
    //Size #size of optimization variables
    iDynTree::VectorDynSize m_preferredJointsConfiguration;

    bool areInitialConditionsSet;
    enum InverseKinematicsTargetResolutionMode targetResolutionMode;

    //Result of optimization
    iDynTree::VectorDynSize m_optimizedRobotDofs;
    iDynTree::Position m_optimizedBasePosition;
    iDynTree::Vector4 m_optimizedBaseOrientation;

    //END Optimization section

    void updateRobotConfiguration();
public:
    InverseKinematicsData();
    
    bool setupFromURDFModelWithFilePath(std::string urdfFilePath);
    bool setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping);

    /**
     * Reset variables to defaults
     *
     * If the model has been loaded, defaults means #dofs size
     * Otherwise the size is zero.
     * All constraints are reset.
     */
    void clearProblem();

    bool addFrameConstraint(const kinematics::Transform& frameTransform);
    bool addTarget(const kinematics::Transform& frameTransform, double weight = 1);

    bool setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration);
    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    
    bool setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration);
    bool setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition);

    void setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization);
    enum InverseKinematicsRotationParametrization rotationParametrization();

    void setTargetResolutionMode(enum InverseKinematicsTargetResolutionMode mode);

    void prepareForOptimization();

    friend class InverseKinematicsNLP;


    iDynTree::HighLevel::DynamicsComputations& dynamics();
    Ipopt::SmartPtr<Ipopt::IpoptApplication> solver;

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H */
