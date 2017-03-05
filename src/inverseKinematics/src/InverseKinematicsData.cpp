/*!
 * @file InverseKinematicsData.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "InverseKinematicsData.h"
#include "Transform.h"
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace internal {
namespace kinematics {

    InverseKinematicsData::InverseKinematicsData(const InverseKinematicsData&) {}
    InverseKinematicsData& InverseKinematicsData::operator=(const InverseKinematicsData&) { return *this; }

    InverseKinematicsData::InverseKinematicsData()
    : m_dofs(0)
    , m_rotationParametrization(iDynTree::InverseKinematicsRotationParametrizationQuaternion)
    , m_areInitialConditionsSet(false)
    , targetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintFull)
    , solver(NULL)
    {
        //These variables are touched only once.
        m_state.worldGravity.zero();
        m_state.worldGravity(3) = -9.81;

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
        m_optimizedRobotDofs.resize(m_dofs);
        m_optimizedRobotDofs.zero();
        m_preferredJointsConfiguration.resize(m_dofs);
        m_preferredJointsConfiguration.zero();

        m_state.jointsConfiguration.resize(m_dofs);
        m_state.jointsConfiguration.zero();

        m_state.jointsVelocity.resize(m_dofs);
        m_state.jointsVelocity.zero();

        m_state.basePose.setPosition(iDynTree::Position(0, 0, 0));
        m_state.basePose.setRotation(iDynTree::Rotation::Identity());

        m_constraints.clear();
        m_targets.clear();

        m_areInitialConditionsSet = false;
    }

    bool InverseKinematicsData::addFrameConstraint(const kinematics::Transform& frameTransform)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        //add the constraint to the set
        std::pair<TransformMap::iterator, bool> result = m_constraints.insert(TransformMap::value_type(frameIndex, frameTransform));
        return result.second;
    }

    bool InverseKinematicsData::addTarget(const kinematics::Transform& frameTransform, double weight)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        std::pair<TransformMap::iterator, bool> result = m_targets.insert(TransformMap::value_type(frameIndex, frameTransform));
        return result.second;
    }

    iDynTree::KinDynComputations& InverseKinematicsData::dynamics() { return m_dynamics; }

    bool InverseKinematicsData::setInitialCondition(const iDynTree::Transform* baseTransform,
                                                    const iDynTree::VectorDynSize* initialJointCondition)
    {
        if (baseTransform) {
            m_optimizedBasePosition = baseTransform->getPosition();
            //if quaternion
            baseTransform->getRotation().getQuaternion(m_optimizedBaseOrientation);
        }
        if (initialJointCondition) {
            m_optimizedRobotDofs = *initialJointCondition;
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

    bool InverseKinematicsData::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration)
    {
        assert(m_optimizedRobotDofs.size() == desiredJointConfiguration.size());
        m_preferredJointsConfiguration = desiredJointConfiguration;
        updateRobotConfiguration();
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
        if (!m_areInitialConditionsSet) {
            for (size_t i = 0; i < m_optimizedRobotDofs.size(); ++i) {
                //check joint to be inside limit
                double jointValue = m_state.jointsConfiguration(i);
                if (jointValue < m_jointLimits[i].first || jointValue > m_jointLimits[i].second) {
                    //set the initial value to be at the middle of the limits
                    jointValue = (m_jointLimits[i].second + m_jointLimits[i].first) / 2.0;
                }
                m_optimizedRobotDofs(i) = jointValue;
            }

            m_optimizedBasePosition = m_state.basePose.getPosition();
            if (m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                m_state.basePose.getRotation().getQuaternion(m_optimizedBaseOrientation);
            } else if (m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                m_state.basePose.getRotation().getRPY(m_optimizedBaseOrientation(0), m_optimizedBaseOrientation(1), m_optimizedBaseOrientation(2));
            }
        }

    }

    void InverseKinematicsData::setTargetResolutionMode(enum iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
        targetResolutionMode = mode;
    }

}
}
