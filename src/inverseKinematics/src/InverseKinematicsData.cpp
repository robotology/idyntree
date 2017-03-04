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

#include <cassert>

namespace internal {
namespace kinematics {

//    InverseKinematicsData::InverseKinematicsData(const InverseKinematicsData&) {}
//    InverseKinematicsData& InverseKinematicsData::operator=(const InverseKinematicsData&) { return *this; }

    InverseKinematicsData::InverseKinematicsData()
    : m_dofs(0)
    , m_rotationParametrization(iDynTree::InverseKinematicsRotationParametrizationQuaternion)
    , areInitialConditionsSet(false)
    , targetResolutionMode(iDynTree::InverseKinematicsTargetResolutionModeFull)
    , solver(NULL)
    {
        //These variables are touched only once.
        m_state.worldGravity.zero();
        m_state.worldGravity(3) = -9.81;

        m_state.baseTwist.zero();
    }

    bool InverseKinematicsData::setupFromURDFModelWithFilePath(std::string urdfFilePath)
    {
        bool result = m_dynamics.loadRobotModelFromFile(urdfFilePath);
        if (!result || !m_dynamics.isValid()) {
            std::cerr << "[ERROR] Error loading URDF model from " << urdfFilePath;
            return false;
        }
        m_dofs = m_dynamics.getNrOfDegreesOfFreedom();

        m_jointLimits.clear();
        m_jointLimits.reserve(m_dofs);
        for (int i = 0; i < m_dofs; i++) {
            std::pair<double, double> limits;
            m_dynamics.getJointLimits(i, limits.first, limits.second);
            m_jointLimits.push_back(limits);
        }

        clearProblem();

        updateRobotConfiguration();

        return true;
    }

    void InverseKinematicsData::clearProblem()
    {
        //resize vectors
        m_state.jointsConfiguration.resize(m_dofs);
        m_state.jointsConfiguration.zero();

        m_state.jointsVelocity.resize(m_dofs);
        m_state.jointsVelocity.zero();

        m_state.basePose.setPosition(iDynTree::Position(0, 0, 0));
        m_state.basePose.setRotation(iDynTree::Rotation::Identity());

        std::vector<std::string> emptyVector;
        this->setOptimizationVariablesToJointsMapping(emptyVector);

        m_constraints.clear();
        m_targets.clear();

        areInitialConditionsSet = false;
    }

    bool InverseKinematicsData::setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping)
    {
//        //reset base
//        m_optimizedBasePosition.zero();
//        if (m_rotationParametrization == InverseKinematicsRotationParametrizationQuaternion) {
//            m_optimizedBaseOrientation = iDynTree::Rotation::Identity().asQuaternion();
//        } else if (m_rotationParametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
//            m_optimizedBaseOrientation.zero();
//            iDynTree::Rotation::Identity().getRPY(m_optimizedBaseOrientation(0), m_optimizedBaseOrientation(1), m_optimizedBaseOrientation(2));
//        }

        unsigned optimizationVariablesSize = m_dofs;

        if (variableToDoFMapping.empty()) {
            //simply remove any mapping
            m_variablesToJointsMapping.clear();
            m_variablesToJointsMapping.reserve(m_dofs);
            for (unsigned i = 0; i < m_dofs; ++i) {
                m_variablesToJointsMapping.push_back(i);
            }
        } else {
            m_variablesToJointsMapping.clear();
            m_variablesToJointsMapping.reserve(variableToDoFMapping.size());
            for (std::vector<std::string>::const_iterator it = variableToDoFMapping.begin();
                 it != variableToDoFMapping.end(); ++it) {
                int jointIndex = m_dynamics.getJointIndex(*it);
                if (jointIndex < 0) {
                    std::cerr << "[ERROR] Could not find joint " << *it << std::endl;
                    return false;
                }
                m_variablesToJointsMapping.push_back(jointIndex);
            }
            optimizationVariablesSize = variableToDoFMapping.size();
        }


        //resize optimization variable
        m_optimizedRobotDofs.resize(optimizationVariablesSize);
        m_optimizedRobotDofs.zero();
        m_preferredJointsConfiguration.resize(optimizationVariablesSize);
        m_preferredJointsConfiguration.zero();

        return true;
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

    bool InverseKinematicsData::setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition)
    {
        if (baseTransform != NULL) {
            m_optimizedBasePosition = baseTransform->getPosition();
            //if quaternion
            baseTransform->getRotation().getQuaternion(m_optimizedBaseOrientation);
        }
        if (initialCondition != NULL) {
            m_optimizedRobotDofs = *initialCondition;
        }
        return true;
    }

    bool InverseKinematicsData::setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration)
    {
        assert(m_state.jointsConfiguration.size() == jointConfiguration.size());
        m_state.jointsConfiguration = jointConfiguration;
        m_state.basePose = baseConfiguration;
        updateRobotConfiguration();
        return true;
    }

    bool InverseKinematicsData::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        int jointIndex = m_dynamics.getJointIndex(jointName);
        if (jointIndex < 0) return false;
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
        if (!areInitialConditionsSet) {
            for (int i = 0; i < m_optimizedRobotDofs.size(); ++i) {
                //check joint to be inside limit
                double jointValue = m_state.jointsConfiguration(m_variablesToJointsMapping[i]);
                if (jointValue < m_jointLimits[i].first || jointValue > m_jointLimits[i].second) {
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

    void InverseKinematicsData::setTargetResolutionMode(enum iDynTree::InverseKinematicsTargetResolutionMode mode)
    {
        targetResolutionMode = mode;
    }

}
}
