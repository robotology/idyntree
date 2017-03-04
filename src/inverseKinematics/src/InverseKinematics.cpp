/*!
 * @file InverseKinematics.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "InverseKinematics.h"
#include "InverseKinematicsData.h"
#include "Transform.h"
#include "InverseKinematicsNLP.h"

#include <iDynTree/Core/Transform.h>

//TEMP:Remove this line
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <iostream>

namespace iDynTree {

    InverseKinematics::InverseKinematics()
    : m_pimpl(0)
    {
        m_pimpl = new InverseKinematicsData();
    }

    InverseKinematics::~InverseKinematics()
    {
        if (m_pimpl) {
            delete m_pimpl;
            m_pimpl = 0;
        }
    }

    bool InverseKinematics::loadModelFromURDFFileWithName(const std::string& urdfFile)
    {
        assert(m_pimpl);

        //First reset the problem
        m_pimpl->clearProblem();

        if (!m_pimpl->setupFromURDFModelWithFilePath(urdfFile))
            return false;

        return true;
    }

    void InverseKinematics::clearProblem()
    {
        assert(m_pimpl);
        m_pimpl->clearProblem();
    }

    bool InverseKinematics::setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName)
    {
        assert(m_pimpl);
        return m_pimpl->dynamics().setFloatingBase(floatingBaseFrameName);
    }

    bool InverseKinematics::setRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration)
    {
        assert(m_pimpl);
        return m_pimpl->setRobotConfiguration(baseConfiguration, jointConfiguration);
    }

    bool InverseKinematics::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        assert(m_pimpl);
        return m_pimpl->setJointConfiguration(jointName, jointConfiguration);
    }

    bool InverseKinematics::setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping)
    {
        assert(m_pimpl);
        return m_pimpl->setOptimizationVariablesToJointsMapping(variableToDoFMapping);
    }

    void InverseKinematics::setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization)
    {
        assert(m_pimpl);
        m_pimpl->setRotationParametrization(parametrization);
    }

    enum InverseKinematicsRotationParametrization InverseKinematics::rotationParametrization()
    {
        assert(m_pimpl);
        return m_pimpl->rotationParametrization();
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName)
    {
        assert(m_pimpl);
        iDynTree::Transform w_X_frame = m_pimpl->dynamics().getWorldTransform(frameName);
        return addFrameConstraint(frameName, w_X_frame);
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::transformConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::positionConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::positionConstraint(frameName, constraintValue.getPosition()));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::rotationConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::rotationConstraint(frameName, constraintValue.getRotation()));
    }

    bool InverseKinematics::addTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::transformConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::positionConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::positionConstraint(frameName,  constraintValue.getPosition()));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::rotationConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::rotationConstraint(frameName,  constraintValue.getRotation()));
    }

    bool InverseKinematics::setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition)
    {
        assert(m_pimpl);
        return m_pimpl->setInitialCondition(baseTransform, initialCondition);
    }

    void InverseKinematics::setTargetResolutionMode(enum InverseKinematicsTargetResolutionMode mode)
    {
        assert(m_pimpl);
        
    }


    bool InverseKinematics::solve()
    {
        assert(m_pimpl);
        //TODO: add error output

        Ipopt::ApplicationReturnStatus solverStatus;

        if (Ipopt::IsNull(m_pimpl->solver)) {
            m_pimpl->solver = IpoptApplicationFactory();

            //TODO: set options
            m_pimpl->solver->Options()->SetStringValue("hessian_approximation", "limited-memory");
//            m_pimpl->solver->Options()->SetIntegerValue("max_iter", 1);
#ifndef NDEBUG
            m_pimpl->solver->Options()->SetStringValue("derivative_test", "first-order");
#endif

            solverStatus = m_pimpl->solver->Initialize();
            if (solverStatus != Ipopt::Solve_Succeeded) {
                return false;
            }
        }

        m_pimpl->prepareForOptimization();

        //instantiate the IpOpt problem
        InverseKinematicsNLP *iKin = new InverseKinematicsNLP(*m_pimpl);
        //Do something (if necessary)
        Ipopt::SmartPtr<Ipopt::TNLP> problem(iKin);


        // Ask Ipopt to solve the problem
        solverStatus = m_pimpl->solver->OptimizeTNLP(problem);

        if (solverStatus == Ipopt::Solve_Succeeded) {
            std::cout << "*** The problem solved!\n";
            return true;
        } else {
            return false;
        }
    }

    bool InverseKinematics::getPoseForFrame(const std::string& frameName, iDynTree::Transform& transform)
    {
        assert(m_pimpl);
        transform = m_pimpl->dynamics().getWorldTransform(frameName);
        return true;

    }

}
