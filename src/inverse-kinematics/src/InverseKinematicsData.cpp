// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "InverseKinematicsData.h"
#include "InverseKinematicsNLP.h"
#include "TransformConstraint.h"
#include <iDynTree/Axis.h>
#include <iDynTree/Twist.h>
#include <iDynTree/ClassicalAcc.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <private/InverseKinematicsData.h>

namespace internal {
namespace kinematics {

    InverseKinematicsData::InverseKinematicsData(const InverseKinematicsData&) {}
    InverseKinematicsData& InverseKinematicsData::operator=(const InverseKinematicsData&) { return *this; }

    InverseKinematicsData::InverseKinematicsData()
    : m_defaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
    , m_dofs(0)
    , m_rotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw)
    , m_areBaseInitialConditionsSet(false)
    , m_areJointsInitialConditionsSet(InverseKinematicsInitialConditionNotSet)
    , m_problemInitialized(false)
    , m_warmStartEnabled(false)
    , m_numberOfOptimisationVariables(0)
    , m_numberOfOptimisationConstraints(0)
    , m_solver(NULL)
    , m_nlpProblem(new internal::kinematics::InverseKinematicsNLP(*this))
    // The default values for the ipopt related parameters are exactly the one of IPOPT,
    // see https://www.coin-or.org/Ipopt/documentation/node41.html
    //     https://www.coin-or.org/Ipopt/documentation/node42.html
    , m_maxIter(3000)
    , m_maxCpuTime(1e6)
    , m_tol(1e-8)
    , m_constrTol(1e-4)
    , m_verbosityLevel(0)
    {
        //These variables are touched only once.
        m_state.worldGravity.zero();

        m_state.baseTwist.zero();
        m_comTarget.isActive = false;
        m_comTarget.weight = 0;
        m_comTarget.desiredPosition.zero();
        m_comTarget.constraintTolerance = 1e-8;
        m_comTarget.isConstraint = false;
    }

    bool InverseKinematicsData::setModel(const iDynTree::Model& model, const std::vector<std::string> &consideredJoints)
    {
        m_dofs = model.getNrOfDOFs();

        m_reducedVariablesInfo.fixedVariables.assign(m_dofs, false);
        m_reducedVariablesInfo.modelJointsToOptimisedJoints.clear();

        if (!consideredJoints.empty()) {

            iDynTree::ModelLoader reducedModelLoader;
            if (!reducedModelLoader.loadReducedModelFromFullModel(model, consideredJoints)){
                std::cerr << "[ERROR] Error loading reduced robot model" << std::endl;
                return false;
            }
            m_reducedVariablesInfo.reducedModel = reducedModelLoader.model();

            for (iDynTree::JointIndex jointIdx = 0; jointIdx < model.getNrOfDOFs(); ++jointIdx) {
                std::string jointName = model.getJointName(jointIdx);
                std::vector<std::string>::const_iterator found = std::find(consideredJoints.begin(), consideredJoints.end(), jointName);
                if (found == consideredJoints.end()) {
                    m_reducedVariablesInfo.fixedVariables[jointIdx] = true;
                    continue;
                }
                // found => get the index in the optimised joints vector
                size_t optimisedIndex = std::distance(consideredJoints.begin(), found);
                m_reducedVariablesInfo.modelJointsToOptimisedJoints.insert(std::unordered_map<int, int>::value_type(optimisedIndex, jointIdx));
            }
        } else {

            m_reducedVariablesInfo.reducedModel = model;

            for (iDynTree::JointIndex jointIdx = 0; jointIdx < model.getNrOfDOFs(); ++jointIdx) {
                m_reducedVariablesInfo.modelJointsToOptimisedJoints.insert(std::unordered_map<int, int>::value_type(jointIdx, jointIdx));
            }
        }

        bool result = m_dynamics.loadRobotModel(model);
        if (!result || !m_dynamics.isValid()) {
            std::cerr << "[ERROR] Error loading robot model" << std::endl;
            return false;
        }

        //prepare joint limits
        m_jointLimits.clear();
        //TODO to be changed to +_ infinity
        //default: no limits
        m_jointLimits.assign(m_dofs, std::pair<double, double>(-2e+19, 2e+19));

        //for each joint, ask the limits
        for (iDynTree::JointIndex jointIdx = 0; jointIdx < model.getNrOfJoints(); ++jointIdx) {
            iDynTree::IJointConstPtr joint = model.getJoint(jointIdx);
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
        m_preferredJointsWeight.resize(m_dofs);
       iDynTree::toEigen(m_preferredJointsWeight).setConstant(1e-6);

        m_state.jointsConfiguration.resize(m_dofs);
        m_state.jointsConfiguration.zero();

        m_state.jointsVelocity.resize(m_dofs);
        m_state.jointsVelocity.zero();

        m_state.basePose.setPosition(iDynTree::Position(0, 0, 0));
        m_state.basePose.setRotation(iDynTree::Rotation::Identity());

        m_constraints.clear();
        m_targets.clear();
        m_comHullConstraint.setActive(false);

        m_areBaseInitialConditionsSet = false;
        m_areJointsInitialConditionsSet = internal::kinematics::InverseKinematicsData::InverseKinematicsInitialConditionNotSet;

        m_comTarget.isActive = false;
        m_comTarget.weight = 0;
        m_comTarget.desiredPosition.zero();
        m_comTarget.constraintTolerance = 1e-8;

        m_problemInitialized = false;
        if (m_warmStartEnabled) {
            m_warmStartEnabled = false;
            if (!Ipopt::IsNull(m_solver)) {
                m_solver->Options()->SetStringValue("warm_start_init_point", "no");
                m_solver->Options()->SetStringValue("warm_start_same_structure", "no");
            }
        }
    }

    bool InverseKinematicsData::addFrameConstraint(const kinematics::TransformConstraint& frameTransformConstraint)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransformConstraint.getFrameName());
        if (frameIndex < 0)
            return false;

        //add the constraint to the set
        std::pair<TransformMap::iterator, bool> result = m_constraints.insert(TransformMap::value_type(frameIndex, frameTransformConstraint));

        // If this method is called again to reconfigure the constraint, the problem needs to be reinitialized
        m_problemInitialized = false;

        return result.second;
    }

    bool InverseKinematicsData::addTarget(const kinematics::TransformConstraint& frameTransform)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        std::pair<TransformMap::iterator, bool> result = m_targets.insert(TransformMap::value_type(frameIndex, frameTransform));
        // As the input is const, I can only modify it after insertion
        if (result.second) {
            result.first->second.setTargetResolutionMode(m_defaultTargetResolutionMode);
        }

        // If this method is called again to reconfigure the constraint, the problem needs to be reinitialized
        m_problemInitialized = false;

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

        switch (m_areJointsInitialConditionsSet) {
            case InverseKinematicsInitialConditionNotSet:
                m_jointInitialConditions = m_state.jointsConfiguration;
                break;
            case InverseKinematicsInitialConditionPartial:
                // in this case we have to set in m_jointInitialConditions
                // the joints in m_state.jointsConfiguration which are not considered
                // in the reduced variables
                for (size_t i = 0; i < m_reducedVariablesInfo.fixedVariables.size(); ++i) {
                    if (!m_reducedVariablesInfo.fixedVariables[i]) continue;
                    // joint is fixed => set the initial condition
                    m_jointInitialConditions(i) = m_state.jointsConfiguration(i);
                }
                break;
            default:
                break;
        }

        //2) Check joint limits..
        for (size_t i = 0; i < m_reducedVariablesInfo.modelJointsToOptimisedJoints.size(); ++i) {
            //check joint to be inside limit
            double &jointValue = m_jointInitialConditions(m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]);
            if (jointValue < m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].first || jointValue > m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].second) {
                std::cerr
                << "[WARNING] InverseKinematics: joint " << m_dynamics.model().getJointName(m_reducedVariablesInfo.modelJointsToOptimisedJoints[i])
                << " (index " << m_reducedVariablesInfo.modelJointsToOptimisedJoints[i] << ") initial condition is outside the limits "
                << m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].first << " " << m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].second
                << ". Actual value: " << jointValue <<  std::endl;
                //set the initial value to at the limit
                if (jointValue < m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].first) {
                    jointValue = m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].first;
                }

                if (jointValue > m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].second) {
                    jointValue = m_jointLimits[m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]].second;
                }
            }
        }

    }

    void InverseKinematicsData::setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
        m_defaultTargetResolutionMode = mode;
    }

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint InverseKinematicsData::defaultTargetResolutionMode()
    {
        return m_defaultTargetResolutionMode;
    }

    void InverseKinematicsData::setTargetResolutionMode(TransformMap::iterator target, iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
       assert(target != m_targets.end());
       target->second.setTargetResolutionMode(mode);
    }

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint InverseKinematicsData::targetResolutionMode(TransformMap::iterator target) const
    {
        assert(target != m_targets.end());
        return target->second.targetResolutionMode();
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
            m_solver->Options()->SetNumericValue("tol", m_tol);
            m_solver->Options()->SetNumericValue("constr_viol_tol", m_constrTol);
            m_solver->Options()->SetIntegerValue("acceptable_iter", 5);
            m_solver->Options()->SetStringValue("fixed_variable_treatment", "make_parameter"); //which btw is the default option
#ifndef NDEBUG
            m_solver->Options()->SetStringValue("derivative_test", "first-order");
#endif
            if (!m_solverName.empty()) {
                m_solver->Options()->SetStringValue("linear_solver", m_solverName);
            } else {
                m_solver->Options()->GetStringValue("linear_solver", m_solverName, "");
            }

            m_solver->Options()->SetNumericValue("warm_start_bound_frac", 1e-6);
            m_solver->Options()->SetNumericValue("warm_start_bound_push", 1e-6);
            m_solver->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-6);
            m_solver->Options()->SetNumericValue("warm_start_slack_bound_frac", 1e-6);
            m_solver->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-6);

            solverStatus = m_solver->Initialize();
            if (solverStatus != Ipopt::Solve_Succeeded) {
                return false;
            }
        }

        if (!m_problemInitialized) {
            computeProblemSizeAndResizeBuffers();
        }

        prepareForOptimization();
        // Ask Ipopt to solve the problem
        solverStatus = m_solver->OptimizeTNLP(m_nlpProblem);

        if (solverStatus == Ipopt::Solve_Succeeded || solverStatus == Ipopt::Solved_To_Acceptable_Level ) {
            if (!m_warmStartEnabled) {
                m_warmStartEnabled = true;
                m_solver->Options()->SetStringValue("warm_start_init_point", "yes");
            }
            return true;
        } else {
            return false;
        }
    }

    void InverseKinematicsData::setCoMTarget(const iDynTree::Position& desiredPosition, double weight){
        this->m_comTarget.desiredPosition = desiredPosition;

        if (!this->m_comTarget.isActive
            && m_defaultTargetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly) {
            this->m_comTarget.isConstraint = true;
        }

        if (!(weight < 0)) {
            this->m_comTarget.weight = weight;
        }

        this->m_comTarget.isActive = true;
    }

    void InverseKinematicsData::setCoMasConstraint(bool asConstraint)
    {
        this->m_comTarget.isConstraint = asConstraint;
    }

    bool InverseKinematicsData::isCoMaConstraint()
    {
        return this->m_comTarget.isConstraint;
    }

    void InverseKinematicsData::setCoMasConstraintTolerance(double TOL)
    {
        this->m_comTarget.constraintTolerance = TOL;
    }

    bool InverseKinematicsData::isCoMTargetActive(){
        return this->m_comTarget.isActive;
    }

    void InverseKinematicsData::setCoMTargetInactive()
    {
        this->m_comTarget.isActive = false;
        this->m_comTarget.weight = 0;
        this->m_comTarget.desiredPosition.zero();
    }

    void InverseKinematicsData::computeProblemSizeAndResizeBuffers()
    {
        //Size of optimization variables is 3 + Orientation (base) + size of joints we optimize
        m_numberOfOptimisationVariables = 3 + sizeOfRotationParametrization(m_rotationParametrization) + m_dofs;

        //Start adding constraints
        m_numberOfOptimisationConstraints = 0;
        for (auto && constraint : m_constraints) {
            //Frame constraint: it can have position, rotation or both elements
            if (constraint.second.isActive()) {
                if (constraint.second.hasPositionConstraint()) {
                    m_numberOfOptimisationConstraints += 3;
                }
                if (constraint.second.hasRotationConstraint()) {
                    m_numberOfOptimisationConstraints += sizeOfRotationParametrization(m_rotationParametrization);
                }
            }
        }

        // COM Convex Hull constraint
        if (m_comHullConstraint.isActive()) {
            this->configureCenterOfMassProjectionConstraint();
            m_numberOfOptimisationConstraints += m_comHullConstraint.getNrOfConstraints();
        }

        if (isCoMTargetActive() && isCoMaConstraint()) {
            m_numberOfOptimisationConstraints += 3;
        }
        //add target if considered as constraints
        for (auto && target : m_targets) {
            if (target.second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                && target.second.hasPositionConstraint()) {
                m_numberOfOptimisationConstraints += 3;
            }
            if (target.second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                && target.second.hasRotationConstraint()) {

                m_numberOfOptimisationConstraints += sizeOfRotationParametrization(m_rotationParametrization);;
            }
        }

        if (m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //If the rotation is parametrized as quaternion
            //that the base orientation yields an additional
            //constraint, i.e. norm of quaternion = 1
            m_numberOfOptimisationConstraints += 1; //quaternion norm constraint
        }

        m_constraintMultipliers.resize(m_numberOfOptimisationConstraints);
        m_constraintMultipliers.zero();
        m_lowerBoundMultipliers.resize(m_numberOfOptimisationVariables);
        m_lowerBoundMultipliers.zero();
        m_upperBoundMultipliers.resize(m_numberOfOptimisationVariables);
        m_upperBoundMultipliers.zero();
        m_nlpProblem->initializeInternalData();

        m_problemInitialized = true;
    }

    void InverseKinematicsData::configureCenterOfMassProjectionConstraint()
    {
        this->m_comHullConstraint.supportFrameIndices.resize(0);
        std::vector<iDynTree::Transform> world_H_support;
        std::vector<iDynTree::Polygon> used_polygons;

        // We iterate on all possible support frames
        for (int i=0; i < m_comHullConstraint_supportFramesIndeces.size(); i++)
        {
            // check for transform
            int frameIndex = m_comHullConstraint_supportFramesIndeces[i];

            internal::kinematics::TransformMap::iterator constraintIt = this->m_constraints.find(frameIndex);

            if (constraintIt->second.isActive())
            {
                // Store the constrained value for this frame
                world_H_support.push_back(constraintIt->second.getTransform());
                used_polygons.push_back(m_comHullConstraint_supportPolygons[i]);
                this->m_comHullConstraint.supportFrameIndices.push_back(frameIndex);
            }
        }

        iDynTree::Axis projectionPlaneXaxisInAbsoluteFrame(m_comHullConstraint_xAxisOfPlaneInWorld, m_comHullConstraint_originOfPlaneInWorld);
        iDynTree::Axis projectionPlaneYaxisInAbsoluteFrame(m_comHullConstraint_yAxisOfPlaneInWorld, m_comHullConstraint_originOfPlaneInWorld);

        // Initialize the COM's projection direction in such a way that is along the lien perpendicular to the xy-axes of the World
        this->m_comHullConstraint.setProjectionAlongDirection(m_comHullConstraint_projDirection);

        // Compute the constraint
        bool ok = this->m_comHullConstraint.buildConvexHull(m_comHullConstraint_xAxisOfPlaneInWorld,
                                                            m_comHullConstraint_yAxisOfPlaneInWorld,
                                                            m_comHullConstraint_originOfPlaneInWorld,
                                                            used_polygons,
                                                            world_H_support);

        // Save some info on the constraints
        this->m_comHullConstraint.absoluteFrame_X_supportFrame = world_H_support;

        return;
    }

    bool InverseKinematicsData::setJointLimits(std::vector<std::pair<double, double> >& jointLimits)
    {
        // check that the dimension of the vector is correct
        if(jointLimits.size() != m_jointLimits.size())
            return false;

        // clear the old limits and assign new limits
        m_jointLimits.clear();
        m_jointLimits = jointLimits;

        return true;
    }

    bool InverseKinematicsData::getJointLimits(std::vector<std::pair<double, double> >& jointLimits)
    {
      // check that the dimension of the vector is correct
      if(jointLimits.size() != m_jointLimits.size())
        return false;

      // return the current limits
      jointLimits = m_jointLimits;

      return true;
    }

}
}
