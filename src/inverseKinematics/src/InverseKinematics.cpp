/*!
 * @file InverseKinematics.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 */

#include "InverseKinematics.h"
#include "InverseKinematicsData.h"
#include "InverseKinematicsNLP.h"
#include "Transform.h"

#include <iDynTree/Core/Transform.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <cassert>
#include <iostream>

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

    bool InverseKinematics::loadModelFromURDFFileWithName(const std::string& urdfFile,
                                                          const std::vector<std::string> &variableToDoFMapping)
    {
        ModelLoader loader;
        if (!loader.loadModelFromFile(urdfFile) || !loader.isValid()) {
            std::cerr << "Failed to load model from URDF file " << urdfFile << std::endl;
            return false;
        }


        if (!variableToDoFMapping.empty()) {
            if (!loader.loadReducedModelFromFullModel(loader.model(), variableToDoFMapping)
                || !loader.isValid()) {
                std::cerr << "Failed to reduce model" << std::endl;
                return false;
            }
        }

        return setModel(loader.model());
    }

    bool InverseKinematics::setModel(const iDynTree::Model &model)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setModel(model);
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

    bool InverseKinematics::addFrameConstraint(const std::string& frameName)
    {
        assert(m_pimpl);
        iDynTree::Transform w_X_frame = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return addFrameConstraint(frameName, w_X_frame);
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::Transform::transformConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::Transform::positionConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::Transform::positionConstraint(frameName, constraintValue.getPosition()));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::Transform::rotationConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::Transform::rotationConstraint(frameName, constraintValue.getRotation()));
    }

    bool InverseKinematics::addTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::Transform::transformConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::Transform::positionConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::Transform::positionConstraint(frameName,  constraintValue.getPosition()));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::Transform::rotationConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::Transform::rotationConstraint(frameName,  constraintValue.getRotation()));
    }

    bool InverseKinematics::setInitialCondition(const iDynTree::Transform* baseTransform, const iDynTree::VectorDynSize* initialCondition)
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setInitialCondition(baseTransform, initialCondition);
    }

    void InverseKinematics::setTargetResolutionMode(enum InverseKinematicsTreatTargetAsConstraint mode)
    {
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->setTargetResolutionMode(mode);
    }

    enum InverseKinematicsTreatTargetAsConstraint InverseKinematics::targetResolutionMode()
    {
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->targetResolutionMode();
    }

    bool InverseKinematics::solve()
    {
        assert(m_pimpl);
        //TODO: add error output

        Ipopt::ApplicationReturnStatus solverStatus;

        if (Ipopt::IsNull(IK_PIMPL(m_pimpl)->solver)) {
            IK_PIMPL(m_pimpl)->solver = IpoptApplicationFactory();

            //TODO: set options
            IK_PIMPL(m_pimpl)->solver->Options()->SetStringValue("hessian_approximation", "limited-memory");
//            m_pimpl->solver->Options()->SetIntegerValue("max_iter", 1);
#ifndef NDEBUG
            IK_PIMPL(m_pimpl)->solver->Options()->SetStringValue("derivative_test", "first-order");
#endif

            solverStatus = IK_PIMPL(m_pimpl)->solver->Initialize();
            if (solverStatus != Ipopt::Solve_Succeeded) {
                return false;
            }
        }

        IK_PIMPL(m_pimpl)->prepareForOptimization();

        //instantiate the IpOpt problem
        internal::kinematics::InverseKinematicsNLP *iKin = new internal::kinematics::InverseKinematicsNLP(*IK_PIMPL(m_pimpl));
        //Do something (if necessary)
        Ipopt::SmartPtr<Ipopt::TNLP> problem(iKin);


        // Ask Ipopt to solve the problem
        solverStatus = IK_PIMPL(m_pimpl)->solver->OptimizeTNLP(problem);

        if (solverStatus == Ipopt::Solve_Succeeded) {
            std::cout << "*** The problem solved!\n";
            return true;
        } else {
            return false;
        }
    }

    bool InverseKinematics::getPoseForFrame(const std::string& frameName,
                                            iDynTree::Transform& transform)
    {
        assert(m_pimpl);
        transform = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return true;

    }

}
