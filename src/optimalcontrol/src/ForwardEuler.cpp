/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
                if (!m_dynamicalSystem_ptr) {
                    return false;
                }

                unsigned int nx = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());
                unsigned int nu = static_cast<unsigned int>(m_dynamicalSystem_ptr->controlSpaceSize());

                m_computationBuffer.resize(nx);

                m_identity.resize(nx, nx);
                toEigen(m_identity) = iDynTreeEigenMatrix::Identity(nx, nx);

                m_stateJacBuffer.resize(nx, nx);
                m_stateJacBuffer.zero();
                m_controlJacBuffer.resize(nx,nu);
                m_controlJacBuffer.zero();
                m_zeroNxNxBuffer.resize(nx,nx);
                m_zeroNxNxBuffer.zero();
                m_zeroNxNuBuffer.resize(nx,nu);
                m_zeroNxNuBuffer.zero();
                m_zeroNuNuBuffer.resize(nu,nu);
                m_zeroNuNuBuffer.zero();

                m_stateHessianBuffer.resize(nx, nx);
                m_stateHessianBuffer.zero();
                m_controlHessianBuffer.resize(nu, nu);
                m_controlHessianBuffer.zero();
                m_mixedHessianBuffer.resize(nx, nu);
                m_mixedHessianBuffer.zero();

                m_lambda.resize(nx);

                m_stateJacobianSparsity.resize(2);
                m_controlJacobianSparsity.resize(2);

                if (m_dynamicalSystem_ptr->dynamicsStateFirstDerivativeSparsity(m_stateJacobianSparsity[0])) {
                    m_stateJacobianSparsity[1].clear();

                    for (size_t i = 0; i < m_dynamicalSystem_ptr->stateSpaceSize(); ++i) {
                        m_stateJacobianSparsity[0].add(i, i);
                        m_stateJacobianSparsity[1].add(i, i);
                    }

                    m_hasStateJacobianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsControlFirstDerivativeSparsity(m_controlJacobianSparsity[0])) {
                    m_controlJacobianSparsity[1].clear();

                    m_hasControlJacobianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateSparsity(m_stateHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_stateHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_stateHessianSparsity[CollocationHessianIndex(1, 1)].clear();

                    m_hasStateHessianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateControlSparsity(m_stateControlHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_stateControlHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_stateControlHessianSparsity[CollocationHessianIndex(1, 0)].clear();
                    m_stateControlHessianSparsity[CollocationHessianIndex(1, 1)].clear();

                    m_hasStateControlHessianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTControlSparsity(m_controlHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_controlHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_controlHessianSparsity[CollocationHessianIndex(1, 1)].clear();

                    m_hasControlHessianSparsity = true;
                }

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

                if (constraintValue.size() != m_dynamicalSystem_ptr->stateSpaceSize()) {
                    constraintValue.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));
                }

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

                if (stateJacobianValues.size() != 2) {
                    stateJacobianValues.resize(2);
                }

                if ((stateJacobianValues[0].rows() != nx) || (stateJacobianValues[0].cols() != nx)) {
                    stateJacobianValues[0].resize(nx,nx);
                }

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

                if ((stateJacobianValues[1].rows() != nx) || (stateJacobianValues[1].cols() != nx)) {
                    stateJacobianValues[1].resize(nx,nx);
                }

                toEigen(stateJacobianValues[1]) = -toEigen(m_identity);

                //Control Jacobians

                if (controlJacobianValues.size() != 2) {
                    controlJacobianValues.resize(2);
                }

                if ((controlJacobianValues[0].rows() != nx) || (controlJacobianValues[0].cols() != nu)) {
                    controlJacobianValues[0].resize(nx,nu);
                }

                if (!(m_dynamicalSystem_ptr->dynamicsControlFirstDerivative(collocationPoints[0], time, m_controlJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system control jacobian.");
                    return false;
                }

                toEigen(controlJacobianValues[0]) = dT*toEigen(m_controlJacBuffer);

                if ((controlJacobianValues[1].rows() != nx) || (controlJacobianValues[1].cols() != nu)) {
                    controlJacobianValues[1].resize(nx,nu);
                }

                controlJacobianValues[1] = m_zeroNxNuBuffer;

                return true;
            }

            bool ForwardEuler::getCollocationConstraintJacobianStateSparsity(std::vector<SparsityStructure> &stateJacobianSparsity)
            {
                if (!m_hasStateJacobianSparsity) {
                    return false;
                }

                stateJacobianSparsity = m_stateJacobianSparsity;
                return true;
            }

            bool ForwardEuler::getCollocationConstraintJacobianControlSparsity(std::vector<SparsityStructure> &controlJacobianSparsity)
            {
                if (!m_hasControlJacobianSparsity) {
                    return false;
                }

                controlJacobianSparsity = m_controlJacobianSparsity;
                return true;
            }

            bool ForwardEuler::evaluateCollocationConstraintSecondDerivatives(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                                              const std::vector<VectorDynSize> &controlInputs, double dT,
                                                                              const VectorDynSize &lambda, CollocationHessianMap &stateSecondDerivative,
                                                                              CollocationHessianMap &controlSecondDerivative, CollocationHessianMap &stateControlSecondDerivative)
            {
                if (!m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives", "Dynamical system not set.");
                    return false;
                }

                if (collocationPoints.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the collocation point does not match the expected one. Input = ";
                    errorMsg << collocationPoints.size() << ", Expected = 2.";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives", errorMsg.str().c_str());
                    return false;
                }

                if (controlInputs.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the control inputs does not match the expected one. Input = ";
                    errorMsg << controlInputs.size() << ", Expected = 2.";
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives", errorMsg.str().c_str());
                    return false;
                }

                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[0])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives", "Error while setting the control input.");
                    return false;
                }

                toEigen(m_lambda) = dT * toEigen(lambda);

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTState(time, collocationPoints[0], m_lambda, m_stateHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system state second derivative.");
                    return false;
                }

                stateSecondDerivative[CollocationHessianIndex(0, 0)] = m_stateHessianBuffer;

                stateSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNxNxBuffer;

                stateSecondDerivative[CollocationHessianIndex(1, 1)] = m_zeroNxNxBuffer;

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTControl(time, collocationPoints[0], m_lambda, m_controlHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system control second derivative.");
                    return false;
                }

                controlSecondDerivative[CollocationHessianIndex(0, 0)] = m_controlHessianBuffer;

                controlSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNuNuBuffer;

                controlSecondDerivative[CollocationHessianIndex(1, 1)] = m_zeroNuNuBuffer;


                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateControl(time, collocationPoints[0], m_lambda, m_mixedHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system second derivative wrt state and control.");
                    return false;
                }

                stateControlSecondDerivative[CollocationHessianIndex(0, 0)] = m_mixedHessianBuffer;

                stateControlSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNxNuBuffer;

                stateControlSecondDerivative[CollocationHessianIndex(1, 0)] = m_zeroNxNuBuffer;

                stateControlSecondDerivative[CollocationHessianIndex(1, 1)] = m_zeroNxNuBuffer;


                return true;
            }

            bool ForwardEuler::getCollocationConstraintSecondDerivativeWRTStateSparsity(CollocationHessianSparsityMap &stateDerivativeSparsity)
            {
                if (!m_hasStateHessianSparsity) {
                    return false;
                }

                stateDerivativeSparsity = m_stateHessianSparsity;
                return true;
            }

            bool ForwardEuler::getCollocationConstraintSecondDerivativeWRTControlSparsity(CollocationHessianSparsityMap &controlDerivativeSparsity)
            {
                if (!m_hasControlHessianSparsity) {
                    return false;
                }

                controlDerivativeSparsity = m_controlHessianSparsity;
                return true;
            }

            bool ForwardEuler::getCollocationConstraintSecondDerivativeWRTStateControlSparsity(CollocationHessianSparsityMap &stateControlDerivativeSparsity)
            {
                if (!m_hasStateControlHessianSparsity) {
                    return false;
                }

                stateControlDerivativeSparsity = m_stateControlHessianSparsity;
                return true;
            }
        }
    }
}
