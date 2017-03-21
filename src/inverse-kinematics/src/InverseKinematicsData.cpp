/*!
 * @file InverseKinematicsData.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#include "InverseKinematicsData.h"
#include "InverseKinematicsNLP.h"
#include "TransformConstraint.h"
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Model/Model.h>

#include <cassert>
#include <private/InverseKinematicsData.h>

namespace internal {
namespace kinematics {

    InverseKinematicsData::InverseKinematicsData(const InverseKinematicsData&) {}
    InverseKinematicsData& InverseKinematicsData::operator=(const InverseKinematicsData&) { return *this; }

    InverseKinematicsData::InverseKinematicsData()
    : m_dofs(0)
    , m_rotationParametrization(iDynTree::InverseKinematicsRotationParametrizationQuaternion)
    , m_targetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintFull)
    , m_areBaseInitialConditionsSet(false)
    , m_areJointsInitialConditionsSet(false)
    , m_solver(NULL)
    // The default values for the ipopt related parameters are exactly the one of IPOPT,
    // see https://www.coin-or.org/Ipopt/documentation/node41.html
    //     https://www.coin-or.org/Ipopt/documentation/node42.html
    , m_maxIter(3000)
    , m_maxCpuTime(1e6)
    , m_tol(1e-8)
    , m_constrTol(1e-4)
    , m_verbosityLevel(5)
    {
        //These variables are touched only once.
        m_state.worldGravity.zero();

        m_state.baseTwist.zero();
    }

    bool InverseKinematicsData::setModel(const iDynTree::Model& model)
    {
        bool result = m_dynamics.loadRobotModel(model);
        if (!result || !m_dynamics.isValid()) {
            std::cerr << "[ERROR] Error loading robot model" << std::endl;
            return false;
        }

        // I don't know if KinDyn can perform some operations on the model
        // For safety I get the model loaded instead of using the input one
        const iDynTree::Model& loadedModel = m_dynamics.model();
        m_dofs = loadedModel.getNrOfDOFs();

        //prepare jiont limits
        m_jointLimits.clear();
        m_jointLimits.resize(m_dofs);
        //TODO to be changed to +_ infinity
        //default: no limits
        m_jointLimits.assign(m_dofs, std::pair<double, double>(-2e+19, 2e+19));

        //for each joint, ask the limits
        for (iDynTree::JointIndex jointIdx = 0; jointIdx < loadedModel.getNrOfJoints(); ++jointIdx) {
            iDynTree::IJointConstPtr joint = loadedModel.getJoint(jointIdx);
            //if the joint does not have limits skip it
            if (!joint->hasPosLimits())
                continue;
            //for each DoF modelled by the joint get the limits
            for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof) {
                if (!joint->getPosLimits(dof,
                                         m_jointLimits[joint->getDOFsOffset() + dof].first,
                                         m_jointLimits[joint->getDOFsOffset() + dof].second))
                    continue;
            }
        }

        //We set a new model, clear the variables
        clearProblem();

        updateRobotConfiguration();

        return true;
    }

    void InverseKinematicsData::clearProblem()
    {
        //resize vectors
        m_jointInitialConditions.resize(m_dofs);
        m_jointInitialConditions.zero();
        m_jointsResults.resize(m_dofs);
        m_jointsResults.zero();
        m_preferredJointsConfiguration.resize(m_dofs);
        m_preferredJointsConfiguration.zero();
        m_preferredJointsWeight = 1e-6;

        m_state.jointsConfiguration.resize(m_dofs);
        m_state.jointsConfiguration.zero();

        m_state.jointsVelocity.resize(m_dofs);
        m_state.jointsVelocity.zero();

        m_state.basePose.setPosition(iDynTree::Position(0, 0, 0));
        m_state.basePose.setRotation(iDynTree::Rotation::Identity());

        m_constraints.clear();
        m_targets.clear();

        m_areBaseInitialConditionsSet = false;
        m_areJointsInitialConditionsSet = false;
    }

    bool InverseKinematicsData::addFrameConstraint(const kinematics::TransformConstraint& frameTransformConstraint)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransformConstraint.getFrameName());
        if (frameIndex < 0)
            return false;

        //add the constraint to the set
        std::pair<TransformMap::iterator, bool> result = m_constraints.insert(TransformMap::value_type(frameIndex, frameTransformConstraint));
        return result.second;
    }

    bool InverseKinematicsData::addTarget(const kinematics::TransformConstraint& frameTransform)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        std::pair<TransformMap::iterator, bool> result = m_targets.insert(TransformMap::value_type(frameIndex, frameTransform));
        return result.second;
    }

    TransformMap::iterator InverseKinematicsData::getTargetRefIfItExists(const std::string targetFrameName)
    {
        // The error for this check is already printed in getFrameIndex
        int frameIndex = m_dynamics.getFrameIndex(targetFrameName);
        if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            return m_targets.end();

        // Find the target (if this fails, it will return m_targets.end()
        return m_targets.find(frameIndex);
    }

    void InverseKinematicsData::updatePositionTarget(TransformMap::iterator target, iDynTree::Position newPos, double newPosWeight)
    {
        assert(target != m_targets.end());
        target->second.setPosition(newPos);
        target->second.setPositionWeight(newPosWeight);
    }

    void InverseKinematicsData::updateRotationTarget(TransformMap::iterator target, iDynTree::Rotation newRot, double newRotWeight)
    {
        assert(target != m_targets.end());
        target->second.setRotation(newRot);
        target->second.setRotationWeight(newRotWeight);
    }

    iDynTree::KinDynComputations& InverseKinematicsData::dynamics() { return m_dynamics; }

    bool InverseKinematicsData::setInitialCondition(const iDynTree::Transform* baseTransform,
                                                    const iDynTree::VectorDynSize* initialJointCondition)
    {
        if (baseTransform) {
            m_baseInitialCondition = *baseTransform;
            m_areBaseInitialConditionsSet = true;
        }
        if (initialJointCondition) {
            m_jointInitialConditions = *initialJointCondition;
            m_areJointsInitialConditionsSet = true;
        }

        return true;
    }

    bool InverseKinematicsData::setRobotConfiguration(const iDynTree::Transform& baseConfiguration,
                                                      const iDynTree::VectorDynSize& jointConfiguration)
    {
        assert(m_state.jointsConfiguration.size() == jointConfiguration.size());
        m_state.jointsConfiguration = jointConfiguration;
        m_state.basePose = baseConfiguration;
        updateRobotConfiguration();
        return true;
    }

    bool InverseKinematicsData::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        iDynTree::JointIndex jointIndex = m_dynamics.model().getJointIndex(jointName);
        if (jointIndex == iDynTree::JOINT_INVALID_INDEX) return false;
        m_state.jointsConfiguration(jointIndex) = jointConfiguration;
        updateRobotConfiguration();
        return true;
    }

    bool InverseKinematicsData::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, const double weight)
    {
        assert(m_preferredJointsConfiguration.size() == desiredJointConfiguration.size());
        m_preferredJointsConfiguration = desiredJointConfiguration;
        if( weight >= 0.0 ) {
            m_preferredJointsWeight = weight;
        }
        return true;
    }

    void InverseKinematicsData::setRotationParametrization(enum iDynTree::InverseKinematicsRotationParametrization parametrization)
    {
        m_rotationParametrization = parametrization;
    }

    enum iDynTree::InverseKinematicsRotationParametrization InverseKinematicsData::rotationParametrization() { return m_rotationParametrization; }

    void InverseKinematicsData::updateRobotConfiguration()
    {
        m_dynamics.setRobotState(m_state.basePose,
                                 m_state.jointsConfiguration,
                                 m_state.baseTwist,
                                 m_state.jointsVelocity,
                                 m_state.worldGravity);
    }

    void InverseKinematicsData::prepareForOptimization()
    {
        //Do all stuff needed before starting an optimization problem
        //1) prepare initial condition if not explicitly set
        if (!m_areBaseInitialConditionsSet) {
            m_baseInitialCondition = m_state.basePose;
        }

        if (!m_areJointsInitialConditionsSet) {
            m_jointInitialConditions = m_state.jointsConfiguration;
        }

        //2) Check joint limits.. Is this necessary?
        for (size_t i = 0; i < m_jointInitialConditions.size(); ++i) {
            //check joint to be inside limit
            double &jointValue = m_jointInitialConditions(i);
            if (jointValue < m_jointLimits[i].first || jointValue > m_jointLimits[i].second) {
                std::cerr << "[WARNING] InverseKinematics: joint with DOFIndex " << i << " initial condition is outside the limits " << m_jointLimits[i].first << " " << m_jointLimits[i].second << std::endl;
                //set the initial value to be at the middle of the limits
                jointValue = (m_jointLimits[i].second + m_jointLimits[i].first) / 2.0;
            }
        }

    }

    void InverseKinematicsData::setTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
        m_targetResolutionMode = mode;
    }

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint InverseKinematicsData::targetResolutionMode()
    {
        return m_targetResolutionMode;
    }

    bool InverseKinematicsData::solveProblem()
    {
        Ipopt::ApplicationReturnStatus solverStatus;

        if (Ipopt::IsNull(m_solver)) {
            m_solver = IpoptApplicationFactory();

            //TODO: set options
            //For example, one needed option is the linear solver type
            //Best thing is to wrap the IPOPT options with new structure so as to abstract them
            m_solver->Options()->SetStringValue("hessian_approximation", "limited-memory");
            m_solver->Options()->SetIntegerValue("print_level",m_verbosityLevel);
            m_solver->Options()->SetIntegerValue("max_iter", m_maxIter);
            m_solver->Options()->SetNumericValue("max_cpu_time", m_maxCpuTime);
            m_solver->Options()->SetNumericValue("tol",m_tol);
            m_solver->Options()->SetNumericValue("constr_viol_tol",m_constrTol);
#ifndef NDEBUG
            m_solver->Options()->SetStringValue("derivative_test", "first-order");
#endif

            solverStatus = m_solver->Initialize();
            if (solverStatus != Ipopt::Solve_Succeeded) {
                return false;
            }
        }

        prepareForOptimization();

        //instantiate the IpOpt problem
        internal::kinematics::InverseKinematicsNLP *iKin = new internal::kinematics::InverseKinematicsNLP(*this);
        //Do something (if necessary)
        Ipopt::SmartPtr<Ipopt::TNLP> problem(iKin);

        // Ask Ipopt to solve the problem
        solverStatus = m_solver->OptimizeTNLP(problem);

        if (solverStatus == Ipopt::Solve_Succeeded || solverStatus == Ipopt::Solved_To_Acceptable_Level ) {
            std::cout << "*** The problem solved!\n";
            return true;
        } else {
            return false;
        }
    }

    void InverseKinematicsData::getSolution(iDynTree::Transform & baseTransformSolution,
                                            iDynTree::VectorDynSize & shapeSolution)
    {
        baseTransformSolution = m_baseResults;
        shapeSolution         = m_jointsResults;
    }

}
}
