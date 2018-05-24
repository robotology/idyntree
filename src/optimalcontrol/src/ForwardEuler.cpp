/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {
namespace optimalcontrol {
namespace integrators {

            bool ForwardEuler::allocateBuffers()
            {
                if (!m_dynamicalSystem_ptr)
                    return false;

                unsigned int nx = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());
                unsigned int nu = static_cast<unsigned int>(m_dynamicalSystem_ptr->controlSpaceSize());

                m_computationBuffer.resize(nx);

                m_identity.resize(nx, nx);
                toEigen(m_identity) = iDynTreeEigenMatrix::Identity(nx, nx);

                m_stateJacBuffer.resize(nx, nx);
                m_controlJacBuffer.resize(nx,nu);
                m_zeroBuffer.resize(nx,nu);
                m_zeroBuffer.zero();

                return true;
            }

            bool ForwardEuler::oneStepIntegration(double t0, double dT, const VectorDynSize &x0, VectorDynSize &x)
            {
                if (!m_dynamicalSystem_ptr) {
                    reportError(m_info.name().c_str(), "oneStepIntegration", "First you have to specify a dynamical system");
                    return false;
                }

                if (x0.size() != m_dynamicalSystem_ptr->stateSpaceSize()){
                    reportError(m_info.name().c_str(), "oneStepIntegration", "Wrong initial state dimension.");
                    return false;
                }

                if (x.size() != m_dynamicalSystem_ptr->stateSpaceSize()){
                    x.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));
                }

                if (!m_dynamicalSystem_ptr->dynamics(x0, t0, m_computationBuffer)){
                    reportError(m_info.name().c_str(), "oneStepIntegration", "Error while evaluating the autonomous dynamics.");
                    return false;
                }

                toEigen(x) = toEigen(x0) + dT * toEigen(m_computationBuffer);

                return true;
            }

            ForwardEuler::ForwardEuler()
            {
                m_infoData->name = "ForwardEuler";
                m_infoData->isExplicit = true;
                m_infoData->numberOfStages = 1;
            }

            ForwardEuler::ForwardEuler(const std::shared_ptr<DynamicalSystem> dynamicalSystem): FixedStepIntegrator(dynamicalSystem)
            {
                m_infoData->name = "ForwardEuler";
                m_infoData->isExplicit = true;
                m_infoData->numberOfStages = 1;
                allocateBuffers();
            }

            ForwardEuler::~ForwardEuler() {}

            bool ForwardEuler::evaluateCollocationConstraint(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                             const std::vector<VectorDynSize> &controlInputs,
                                                             double dT, VectorDynSize &constraintValue)
            {
                if (!m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Dynamical system not set.");
                    return false;
                }

                if (collocationPoints.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the collocation point does not match the expected one. Input = ";
                    errorMsg << collocationPoints.size() << ", Expected = 2.";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", errorMsg.str().c_str());
                    return false;
                }

                if (controlInputs.size() < 1){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the control inputs does not match the expected one. Input = ";
                    errorMsg << controlInputs.size() << ", Expected = 1 (at least).";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", errorMsg.str().c_str());
                    return false;
                }

                if (constraintValue.size() != m_dynamicalSystem_ptr->stateSpaceSize())
                    constraintValue.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));

                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[0])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Error while setting the control input.");
                    return false;
                }

                if (!(m_dynamicalSystem_ptr->dynamics(collocationPoints[0], time, m_computationBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Error while evaluating the dynamical system.");
                    return false;
                }

                toEigen(constraintValue) = -toEigen(collocationPoints[1]) + toEigen(collocationPoints[0]) +
                        dT*toEigen(m_computationBuffer);

                return true;
            }

            bool ForwardEuler::evaluateCollocationConstraintJacobian(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                                     const std::vector<VectorDynSize> &controlInputs, double dT,
                                                                     std::vector<MatrixDynSize> &stateJacobianValues,
                                                                     std::vector<MatrixDynSize> &controlJacobianValues)
            {
                if (!m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Dynamical system not set.");
                    return false;
                }

                if (collocationPoints.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the collocation point does not match the expected one. Input = ";
                    errorMsg << collocationPoints.size() << ", Expected = 2.";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian", errorMsg.str().c_str());
                    return false;
                }

                if (controlInputs.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the control inputs does not match the expected one. Input = ";
                    errorMsg << controlInputs.size() << ", Expected = 2.";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian", errorMsg.str().c_str());
                    return false;
                }

                unsigned int nx = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());
                unsigned int nu = static_cast<unsigned int>(m_dynamicalSystem_ptr->controlSpaceSize());

                if (stateJacobianValues.size() != 2)
                    stateJacobianValues.resize(2);

                if ((stateJacobianValues[0].rows() != nx) || (stateJacobianValues[0].cols() != nx))
                    stateJacobianValues[0].resize(nx,nx);

                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[0])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian", "Error while setting the control input.");
                    return false;
                }

                if (!(m_dynamicalSystem_ptr->dynamicsStateFirstDerivative(collocationPoints[0], time, m_stateJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system jacobian.");
                    return false;
                }

                toEigen(stateJacobianValues[0]) = toEigen(m_identity) + dT*toEigen(m_stateJacBuffer);

                if ((stateJacobianValues[1].rows() != nx) || (stateJacobianValues[1].cols() != nx))
                    stateJacobianValues[1].resize(nx,nx);

                toEigen(stateJacobianValues[1]) = -toEigen(m_identity);

                //Control Jacobians

                if (controlJacobianValues.size() != 2)
                    controlJacobianValues.resize(2);

                if ((controlJacobianValues[0].rows() != nx) || (controlJacobianValues[0].cols() != nu))
                    controlJacobianValues[0].resize(nx,nu);

                if (!(m_dynamicalSystem_ptr->dynamicsControlFirstDerivative(collocationPoints[0], time, m_controlJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system control jacobian.");
                    return false;
                }

                toEigen(controlJacobianValues[0]) = dT*toEigen(m_controlJacBuffer);

                if ((controlJacobianValues[1].rows() != nx) || (controlJacobianValues[1].cols() != nu))
                    controlJacobianValues[1].resize(nx,nu);

                controlJacobianValues[1] = m_zeroBuffer;

                return true;
            }
        }
    }
}
