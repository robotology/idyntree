/*!
 * @file InverseKinematics.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 */

#include <iDynTree/InverseKinematics.h>
#include "InverseKinematicsData.h"
#include "TransformConstraint.h"

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <iostream>

// TODO: directly access the raw data, thus removing the methods in IKData class
#define IK_PIMPL(x) static_cast<internal::kinematics::InverseKinematicsData*>((x))

/*
 * Private implementation is divided in two classes.
 * - IKData is responsible of handling all the data and functions which are
 *   tied to a particular solver
 * - IKNLP is an IPOPT NLP implementation. It manages only IPOPT related data
 *   and implements IPOPT related functions
 */

namespace iDynTree {

    InverseKinematics::InverseKinematics()
    : m_pimpl(0)
    {
        m_pimpl = new internal::kinematics::InverseKinematicsData();
    }

    InverseKinematics::~InverseKinematics()
    {
        if (m_pimpl) {
            delete IK_PIMPL(m_pimpl);
            m_pimpl = 0;
        }
    }

    bool InverseKinematics::loadModelFromFile(const std::string & filename,
                                              const std::vector<std::string> &consideredJoints,
                                              const std::string & filetype)
    {
        ModelLoader loader;
        if (!loader.loadModelFromFile(filename) || !loader.isValid()) {
            std::cerr << "[ERROR] iDynTree::InverseDynamics : Failed to load model from URDF file " << filename << std::endl;
            return false;
        }

        return setModel(loader.model(), consideredJoints);
    }

    bool InverseKinematics::setModel(const iDynTree::Model &model,
                                     const std::vector<std::string> &consideredJoints)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setModel(model, consideredJoints);
    }

    void InverseKinematics::clearProblem()
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->clearProblem();
    }

    bool InverseKinematics::setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->dynamics().setFloatingBase(floatingBaseFrameName);
    }

    bool InverseKinematics::setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setRobotConfiguration(baseConfiguration, jointConfiguration);
    }

    bool InverseKinematics::setCurrentRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setRobotConfiguration(baseConfiguration, jointConfiguration);
    }


    bool InverseKinematics::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setJointConfiguration(jointName, jointConfiguration);
    }

    void InverseKinematics::setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization)
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->setRotationParametrization(parametrization);
    }

    enum InverseKinematicsRotationParametrization InverseKinematics::rotationParametrization()
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->rotationParametrization();
    }

    void InverseKinematics::setMaxIterations(const int max_iter)
    {
        if (max_iter>0)
            IK_PIMPL(m_pimpl)->m_maxIter = max_iter;
        else
            IK_PIMPL(m_pimpl)->m_maxIter = std::numeric_limits<int>::max();

    }

    int InverseKinematics::maxIterations() const
    {
        return IK_PIMPL(m_pimpl)->m_maxIter;
    }

    void InverseKinematics::setMaxCPUTime(const double max_cpu_time)
    {
        IK_PIMPL(m_pimpl)->m_maxCpuTime = max_cpu_time;
    }

    double InverseKinematics::maxCPUTime() const
    {
        return IK_PIMPL(m_pimpl)->m_maxCpuTime;
    }

    void InverseKinematics::setCostTolerance(const double tol)
    {
        IK_PIMPL(m_pimpl)->m_tol = tol;
    }

    double InverseKinematics::costTolerance() const
    {
        return IK_PIMPL(m_pimpl)->m_tol;
    }

    void InverseKinematics::setConstraintsTolerance(const double constr_tol)
    {
        IK_PIMPL(m_pimpl)->m_constrTol = constr_tol;
    }

    double InverseKinematics::constraintsTolerance() const
    {
        return IK_PIMPL(m_pimpl)->m_constrTol;
    }

    void InverseKinematics::setVerbosity(const unsigned int verbose)
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->m_verbosityLevel = verbose;
    }

    std::string InverseKinematics::linearSolverName()
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_solverName;
    }

    void InverseKinematics::setLinearSolverName(const std::string &solverName)
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->m_solverName = solverName;
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName)
    {
        assert(m_pimpl);
        iDynTree::Transform w_X_frame = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return addFrameConstraint(frameName, w_X_frame);
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::fullTransformConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::positionConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::positionConstraint(frameName, constraintValue.getPosition()));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::rotationConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::rotationConstraint(frameName, constraintValue.getRotation()));
    }

    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::string &firstSupportFrame,
                                                                const Polygon &firstSupportPolygon,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
        std::vector<std::string> supportFrames;
        std::vector<Polygon> supportPolygons;
        supportFrames.push_back(firstSupportFrame);
        supportPolygons.push_back(firstSupportPolygon);
        return addCenterOfMassProjectionConstraint(supportFrames,supportPolygons,xAxisOfPlaneInWorld,yAxisOfPlaneInWorld,originOfPlaneInWorld);
    }

    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::string &firstSupportFrame,
                                                                const Polygon &firstSupportPolygon,
                                                                const std::string &secondSupportFrame,
                                                                const Polygon &secondSupportPolygon,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
        std::vector<std::string> supportFrames;
        std::vector<Polygon> supportPolygons;
        supportFrames.push_back(firstSupportFrame);
        supportFrames.push_back(secondSupportFrame);
        supportPolygons.push_back(firstSupportPolygon);
        supportPolygons.push_back(secondSupportPolygon);
        return addCenterOfMassProjectionConstraint(supportFrames,supportPolygons,xAxisOfPlaneInWorld,yAxisOfPlaneInWorld,originOfPlaneInWorld);
    }

    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::vector<std::string> &supportFrames,
                                                                const std::vector<Polygon> &supportPolygons,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
        if( supportFrames.size() == 0 )
        {
            reportError("InverseKinematics","addCenterOfMassProjectionConstraint","No support frames specified");
            return false;
        }

        if( supportFrames.size() != supportPolygons.size() )
        {
            reportError("InverseKinematics","addCenterOfMassProjectionConstraint","Size mismatch between supportFrames and supportPolygons");
            return false;
        }

        size_t nrOfSupportLinks = supportFrames.size();

        IK_PIMPL(m_pimpl)->m_comHullConstraint.supportFrameIndices.resize(0);
        std::vector<iDynTree::Transform> world_H_support(nrOfSupportLinks);

        for (int i=0; i < nrOfSupportLinks; i++)
        {
            // check for transform
            int frameIndex = IK_PIMPL(m_pimpl)->m_dynamics.getFrameIndex(supportFrames[i]);
            if (frameIndex  == iDynTree::FRAME_INVALID_INDEX)
            {
                std::stringstream ss;
                ss << "Frame " << supportFrames[i] << " not found in the model";
                reportError("InverseKinematics","addCenterOfMassProjectionConstraint",ss.str().c_str());
                return false;
            }

            internal::kinematics::TransformMap::iterator constraintIt = IK_PIMPL(m_pimpl)->m_constraints.find(frameIndex);
            if (constraintIt == IK_PIMPL(m_pimpl)->m_constraints.end())
            {
                std::stringstream ss;
                ss << "Frame " << supportFrames[i] << " is not subject to a constraint";
                reportError("InverseKinematics","addCenterOfMassProjectionConstraint",ss.str().c_str());
                return false;
            }

            IK_PIMPL(m_pimpl)->m_comHullConstraint.supportFrameIndices.push_back(frameIndex);

            // Store the constrained value for this frame
            world_H_support[i] = constraintIt->second.getTransform();
        }

        iDynTree::Axis projectionPlaneXaxisInAbsoluteFrame(xAxisOfPlaneInWorld,originOfPlaneInWorld);
        iDynTree::Axis projectionPlaneYaxisInAbsoluteFrame(yAxisOfPlaneInWorld,originOfPlaneInWorld);
	
	// Initialize the COM's projection direction in such a way that is along the lien perpendicular to the xy-axes of the World
	iDynTree::Direction zAxisOfPlaneInWorld;
	toEigen(zAxisOfPlaneInWorld) = toEigen(xAxisOfPlaneInWorld).cross(toEigen(yAxisOfPlaneInWorld));
		
	IK_PIMPL(m_pimpl)->m_comHullConstraint.setProjectionAlongDirection(zAxisOfPlaneInWorld);

        // Compute the constraint
        bool ok = IK_PIMPL(m_pimpl)->m_comHullConstraint.buildConvexHull(xAxisOfPlaneInWorld,
                                                                         yAxisOfPlaneInWorld,
                                                                         originOfPlaneInWorld,
                                                                         supportPolygons,
                                                                         world_H_support);

        // Save some info on the constraints
        IK_PIMPL(m_pimpl)->m_comHullConstraint.absoluteFrame_X_supportFrame = world_H_support;

        if (!ok)
        {
            reportError("InverseKinematics","addCenterOfMassProjectionConstraint","Problem in computing COM constraint matrices.");
            return false;
        }

        // Configuration went fine, enable constraint
        IK_PIMPL(m_pimpl)->m_comHullConstraint.setActive(true);

        return true;
    }

    double InverseKinematics::getCenterOfMassProjectionMargin()
    {
        // Compute center of mass in the first constraint frame
        iDynTree::KinDynComputations & kinDyn = IK_PIMPL(m_pimpl)->m_dynamics;
        iDynTree::Position comInAbsoluteConstraintFrame =
            IK_PIMPL(m_pimpl)->m_comHullConstraint.absoluteFrame_X_supportFrame[0]*(kinDyn.getWorldTransform(IK_PIMPL(m_pimpl)->m_comHullConstraint.supportFrameIndices[0]).inverse()*kinDyn.getCenterOfMassPosition());

        //iDynTree::Vector2 comProjection = IK_PIMPL(m_pimpl)->m_comHullConstraint.project(comInAbsoluteConstraintFrame);
        iDynTree::Vector2 comProjection = IK_PIMPL(m_pimpl)->m_comHullConstraint.projectAlongDirection(comInAbsoluteConstraintFrame);
        return IK_PIMPL(m_pimpl)->m_comHullConstraint.computeMargin(comProjection);
    }

    bool InverseKinematics::addTarget(const std::string& frameName,
                                      const iDynTree::Transform& constraintValue,
                                      const double positionWeight,
                                      const double rotationWeight)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::fullTransformConstraint(frameName,
                                                                                                               constraintValue,
                                                                                                               positionWeight,
                                                                                                               rotationWeight));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue, const double positionWeight)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::positionConstraint(frameName,  constraintValue, positionWeight));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue, const double positionWeight)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::positionConstraint(frameName,  constraintValue.getPosition(), positionWeight));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue, const double rotationWeight)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::rotationConstraint(frameName,  constraintValue, rotationWeight));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue, const double rotationWeight)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::rotationConstraint(frameName,  constraintValue.getRotation(), rotationWeight));
    }

    bool InverseKinematics::updateTarget(const std::string& frameName,
                                         const Transform& targetValue,
                                         const double positionWeight,
                                         const double rotationWeight)
    {
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updateTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updatePositionTarget(transConstr,targetValue.getPosition(),positionWeight);
        IK_PIMPL(m_pimpl)->updateRotationTarget(transConstr,targetValue.getRotation(),rotationWeight);
        return true;
    }

    bool InverseKinematics::updatePositionTarget(const std::string& frameName,
                                                 const Position& targetValue,
                                                 const double positionWeight)
    {
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updatePositionTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updatePositionTarget(transConstr,targetValue,positionWeight);
        return true;
    }

    bool InverseKinematics::updateRotationTarget(const std::string& frameName,
                                                 const Rotation& targetValue,
                                                 const double rotationWeight)
    {
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updateRotationTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updateRotationTarget(transConstr,targetValue,rotationWeight);
        return true;
    }

    bool InverseKinematics::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, double weight)
    {
        return this->setDesiredReducedJointConfiguration(desiredJointConfiguration, weight);
    }

    bool InverseKinematics::setDesiredFullJointsConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, double weight)
    {
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration.size() == desiredJointConfiguration.size());
        IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration = desiredJointConfiguration;
        if (weight >= 0.0) {
            IK_PIMPL(m_pimpl)->m_preferredJointsWeight = weight;
        }
        return true;
    }

    bool InverseKinematics::setDesiredReducedJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration, double weight)
    {
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size() == desiredJointConfiguration.size());
        for (size_t i = 0; i < desiredJointConfiguration.size(); ++i) {
            IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = desiredJointConfiguration(i);
        }

        if (weight >= 0.0) {
            IK_PIMPL(m_pimpl)->m_preferredJointsWeight = weight;
        }
        return true;
    }

    bool InverseKinematics::setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition)
    {
        return this->setReducedInitialCondition(baseTransform, initialCondition);
    }

    bool InverseKinematics::setFullJointsInitialCondition(const iDynTree::Transform* baseTransform,
                                                          const iDynTree::VectorDynSize* initialCondition)
    {
        assert(m_pimpl);
        if (baseTransform) {
            IK_PIMPL(m_pimpl)->m_baseInitialCondition = *baseTransform;
            IK_PIMPL(m_pimpl)->m_areBaseInitialConditionsSet = true;
        }
        if (initialCondition) {
            assert(initialCondition->size() == IK_PIMPL(m_pimpl)->m_jointInitialConditions.size());
            IK_PIMPL(m_pimpl)->m_jointInitialConditions = *initialCondition;
            IK_PIMPL(m_pimpl)->m_areJointsInitialConditionsSet = internal::kinematics::InverseKinematicsData::InverseKinematicsInitialConditionFull;
        }
        return true;
    }

    bool InverseKinematics::setReducedInitialCondition(const iDynTree::Transform* baseTransform,
                                                       const iDynTree::VectorDynSize* initialCondition)
    {
        assert(m_pimpl);
        if (baseTransform) {
            IK_PIMPL(m_pimpl)->m_baseInitialCondition = *baseTransform;
            IK_PIMPL(m_pimpl)->m_areBaseInitialConditionsSet = true;
        }
        if (initialCondition) {
            // This function passes a guess for the ik for the optimised joints only
            assert(initialCondition->size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size());
            for (size_t i = 0; i < initialCondition->size(); ++i) {
                IK_PIMPL(m_pimpl)->m_jointInitialConditions(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = (*initialCondition)(i);
            }
            IK_PIMPL(m_pimpl)->m_areJointsInitialConditionsSet = internal::kinematics::InverseKinematicsData::InverseKinematicsInitialConditionPartial;
        }
        return true;
    }

    void InverseKinematics::setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->setDefaultTargetResolutionMode(mode);
    }

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint InverseKinematics::defaultTargetResolutionMode()
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->defaultTargetResolutionMode();
    }
    
    bool InverseKinematics::setTargetResolutionMode(const std::string& frameName, InverseKinematicsTreatTargetAsConstraint mode)
    {
        assert(m_pimpl);
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);
        
        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","setTargetResolutionMode",ss.str().c_str());
            return false;
        }
        
        IK_PIMPL(m_pimpl)->setTargetResolutionMode(transConstr, mode);
        return true;
    }


    enum InverseKinematicsTreatTargetAsConstraint InverseKinematics::targetResolutionMode(const std::string& frameName)
    {
        assert(m_pimpl);
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);
        
        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","targetResolutionMode",ss.str().c_str());
            return InverseKinematicsTreatTargetAsConstraintNone;
        }
        
        return IK_PIMPL(m_pimpl)->targetResolutionMode(transConstr);
    }

    bool InverseKinematics::solve()
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->solveProblem();
    }

    void InverseKinematics::getSolution(iDynTree::Transform & baseTransformSolution,
                                        iDynTree::VectorDynSize & shapeSolution)
    {
        this->getReducedSolution(baseTransformSolution, shapeSolution);
        return;
    }

    void InverseKinematics::getFullJointsSolution(iDynTree::Transform & baseTransformSolution,
                                                  iDynTree::VectorDynSize & shapeSolution)
    {
        assert(m_pimpl);
        assert(shapeSolution.size() == IK_PIMPL(m_pimpl)->m_dofs);
        baseTransformSolution = IK_PIMPL(m_pimpl)->m_baseResults;
        shapeSolution         = IK_PIMPL(m_pimpl)->m_jointsResults;
    }

    void InverseKinematics::getReducedSolution(iDynTree::Transform & baseTransformSolution,
                                               iDynTree::VectorDynSize & shapeSolution)
    {
        assert(m_pimpl);
        baseTransformSolution = IK_PIMPL(m_pimpl)->m_baseResults;
        assert(shapeSolution.size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size());
        for (size_t i = 0; i < shapeSolution.size(); ++i) {
            shapeSolution(i) = IK_PIMPL(m_pimpl)->m_jointsResults(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]);
        }
        return;
    }

    bool InverseKinematics::getPoseForFrame(const std::string& frameName,
                                            iDynTree::Transform& transform)
    {
        assert(m_pimpl);
        transform = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return true;

    }

    const Model& InverseKinematics::model() const
    {
        assert(m_pimpl);
        return this->reducedModel();
    }

    const Model& InverseKinematics::fullModel() const
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_dynamics.model();
    }
    
    const Model& InverseKinematics::reducedModel() const
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.reducedModel;
    }
    
    bool InverseKinematics::isCOMTargetActive()
    {
        return IK_PIMPL(m_pimpl)->isCoMTargetActive();
    }
    
    void InverseKinematics::setCOMAsConstraint(bool asConstraint)
    {
        IK_PIMPL(m_pimpl)->setCoMasConstraint(asConstraint);
    }

    
    bool InverseKinematics::isCOMAConstraint()
    {
        return IK_PIMPL(m_pimpl)->isCoMaConstraint();
    }

    void InverseKinematics::setCOMTarget(Position& desiredPosition, double weight)
    {
        IK_PIMPL(m_pimpl)->setCoMTarget(desiredPosition, weight);
    }

    void InverseKinematics::setCOMAsConstraintTolerance(double tolerance)
    {
        IK_PIMPL(m_pimpl)->setCoMasConstraintTolerance(tolerance);
    }


    void InverseKinematics::deactivateCOMTarget()
    {
        IK_PIMPL(m_pimpl)->setCoMTargetInactive();
    }

    void InverseKinematics::setCOMConstraintProjectionDirection(iDynTree::Vector3 direction)
    {
        // define the projection matrix 'Pdirection' in the class 'ConvexHullProjectionConstraint'
        IK_PIMPL(m_pimpl)->m_comHullConstraint.setProjectionAlongDirection(direction);
    }

}
