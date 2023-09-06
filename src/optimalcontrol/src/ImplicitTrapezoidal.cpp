// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Integrators/ImplicitTrapezoidal.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Utils.h>
#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>
#include <cstddef>
#include <sstream>
#include <string>

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {

            bool ImplicitTrapezoidal::allocateBuffers()
            {
                if (!m_dynamicalSystem_ptr) {
                    return false;
                }

                m_computationBuffer.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));
                m_computationBuffer2.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));

                unsigned int nx = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());
                unsigned int nu = static_cast<unsigned int>(m_dynamicalSystem_ptr->controlSpaceSize());

                m_identity.resize(nx, nx);
                toEigen(m_identity) = iDynTreeEigenMatrix::Identity(nx, nx);

                m_stateJacBuffer.resize(nx, nx);
                m_stateJacBuffer.zero();
                m_controlJacBuffer.resize(nx,nu);
                m_controlJacBuffer.zero();

                m_stateJacobianSparsity.resize(2);
                m_controlJacobianSparsity.resize(2);

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

                if (m_dynamicalSystem_ptr->dynamicsStateFirstDerivativeSparsity(m_stateJacobianSparsity[0])) {

                    for (size_t i = 0; i < m_dynamicalSystem_ptr->stateSpaceSize(); ++i) {
                        m_stateJacobianSparsity[0].add(i, i);
                    }
                    m_stateJacobianSparsity[1] = m_stateJacobianSparsity[0];

                    m_hasStateJacobianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsControlFirstDerivativeSparsity(m_controlJacobianSparsity[0])) {

                    m_controlJacobianSparsity[1] = m_controlJacobianSparsity[0];

                    m_hasControlJacobianSparsity = true;

                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateSparsity(m_stateHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_stateHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_stateHessianSparsity[CollocationHessianIndex(1, 1)] = m_stateHessianSparsity[CollocationHessianIndex(0, 0)];

                    m_hasStateHessianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateControlSparsity(m_stateControlHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_stateControlHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_stateControlHessianSparsity[CollocationHessianIndex(1, 0)].clear();
                    m_stateControlHessianSparsity[CollocationHessianIndex(1, 1)] = m_stateControlHessianSparsity[CollocationHessianIndex(0, 0)];

                    m_hasStateControlHessianSparsity = true;
                }

                if (m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTControlSparsity(m_controlHessianSparsity[CollocationHessianIndex(0, 0)])) {
                    m_controlHessianSparsity[CollocationHessianIndex(0, 1)].clear();
                    m_controlHessianSparsity[CollocationHessianIndex(1, 1)] = m_controlHessianSparsity[CollocationHessianIndex(0, 0)];

                    m_hasControlHessianSparsity = true;
                }

                return true;
            }

            bool ImplicitTrapezoidal::oneStepIntegration(double /*t0*/, double /*dT*/, const iDynTree::VectorDynSize &/*x0*/, iDynTree::VectorDynSize &/*x*/)
            {
                reportError(m_info.name().c_str(), "oneStepIntegration", "The ImplicitTrapezoidal method has not been implemented to integrate a dynamical system yet.");
                return false;
            }

            ImplicitTrapezoidal::ImplicitTrapezoidal()
            {
                m_infoData->name = "ImplicitTrapezoidal";
                m_infoData->isExplicit = false;
                m_infoData->numberOfStages = 1;
            }

            ImplicitTrapezoidal::ImplicitTrapezoidal(const std::shared_ptr<DynamicalSystem> dynamicalSystem) : FixedStepIntegrator(dynamicalSystem)
            {
                m_infoData->name = "ImplicitTrapezoidal";
                m_infoData->isExplicit = false;
                m_infoData->numberOfStages = 1;
                allocateBuffers();
            }

            ImplicitTrapezoidal::~ImplicitTrapezoidal()
            {}

            bool ImplicitTrapezoidal::evaluateCollocationConstraint(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                                    const std::vector<VectorDynSize> &controlInputs, double dT,
                                                                    VectorDynSize &constraintValue)
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

                if (controlInputs.size() != 2){
                    std::ostringstream errorMsg;
                    errorMsg << "The size of the matrix containing the control inputs does not match the expected one. Input = ";
                    errorMsg << controlInputs.size() << ", Expected = 2.";
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

                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[1])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Error while setting the control input.");
                    return false;
                }

                if (!(m_dynamicalSystem_ptr->dynamics(collocationPoints[1], time+dT, m_computationBuffer2))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraint", "Error while evaluating the dynamical system.");
                    return false;
                }

                toEigen(constraintValue) = -toEigen(collocationPoints[1]) + toEigen(collocationPoints[0]) +
                        dT/2.0*(toEigen(m_computationBuffer)+toEigen(m_computationBuffer2));

                return true;
            }

            bool ImplicitTrapezoidal::evaluateCollocationConstraintJacobian(double time, const std::vector<VectorDynSize> &collocationPoints,
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

                if (controlJacobianValues.size() != 2) {
                    controlJacobianValues.resize(2);
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

                toEigen(stateJacobianValues[0]) = toEigen(m_identity) + dT/2*toEigen(m_stateJacBuffer);


                if ((controlJacobianValues[0].rows() != nx) || (controlJacobianValues[0].cols() != nu)) {
                    controlJacobianValues[0].resize(nx,nu);
                }


                if (!(m_dynamicalSystem_ptr->dynamicsControlFirstDerivative(collocationPoints[0], time, m_controlJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system control jacobian.");
                    return false;
                }

                toEigen(controlJacobianValues[0]) = dT/2.0*toEigen(m_controlJacBuffer);

                if ((stateJacobianValues[1].rows() != nx) || (stateJacobianValues[1].cols() != nx)) {
                    stateJacobianValues[1].resize(nx,nx);
                }

                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[1])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian", "Error while setting the control input.");
                    return false;
                }

                if (!(m_dynamicalSystem_ptr->dynamicsStateFirstDerivative(collocationPoints[1], time+dT, m_stateJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system jacobian.");
                    return false;
                }

                toEigen(stateJacobianValues[1]) = -toEigen(m_identity) + dT/2*toEigen(m_stateJacBuffer);


                if ((controlJacobianValues[1].rows() != nx) || (controlJacobianValues[1].cols() != nu)) {
                    controlJacobianValues[1].resize(nx,nu);
                }

                if (!(m_dynamicalSystem_ptr->dynamicsControlFirstDerivative(collocationPoints[1], time + dT, m_controlJacBuffer))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintJacobian",
                                "Error while evaluating the dynamical system control jacobian.");
                    return false;
                }

                toEigen(controlJacobianValues[1]) = dT/2.0*toEigen(m_controlJacBuffer);

                return true;

            }

            bool ImplicitTrapezoidal::getCollocationConstraintJacobianStateSparsity(std::vector<SparsityStructure> &stateJacobianSparsity)
            {
                if (!m_hasStateJacobianSparsity) {
                    return false;
                }

                stateJacobianSparsity = m_stateJacobianSparsity;
                return true;
            }

            bool ImplicitTrapezoidal::getCollocationConstraintJacobianControlSparsity(std::vector<SparsityStructure> &controlJacobianSparsity)
            {
                if (!m_hasControlJacobianSparsity) {
                    return false;
                }

                controlJacobianSparsity = m_controlJacobianSparsity;
                return true;
            }

            bool ImplicitTrapezoidal::evaluateCollocationConstraintSecondDerivatives(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                                                     const std::vector<VectorDynSize> &controlInputs, double dT, const VectorDynSize &lambda,
                                                                                     iDynTree::optimalcontrol::integrators::CollocationHessianMap &stateSecondDerivative,
                                                                                     iDynTree::optimalcontrol::integrators::CollocationHessianMap &controlSecondDerivative,
                                                                                     iDynTree::optimalcontrol::integrators::CollocationHessianMap &stateControlSecondDerivative)
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

                toEigen(m_lambda) = 0.5 * dT * toEigen(lambda);

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTState(time, collocationPoints[0], m_lambda, m_stateHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system state second derivative.");
                    return false;
                }

                stateSecondDerivative[CollocationHessianIndex(0, 0)] = m_stateHessianBuffer;

                stateSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNxNxBuffer;

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTControl(time, collocationPoints[0], m_lambda, m_controlHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system control second derivative.");
                    return false;
                }

                controlSecondDerivative[CollocationHessianIndex(0, 0)] = m_controlHessianBuffer;

                controlSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNuNuBuffer;

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateControl(time, collocationPoints[0], m_lambda, m_mixedHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system second derivative wrt state and control.");
                    return false;
                }

                stateControlSecondDerivative[CollocationHessianIndex(0, 0)] = m_mixedHessianBuffer;

                stateControlSecondDerivative[CollocationHessianIndex(0, 1)] = m_zeroNxNuBuffer;

                stateControlSecondDerivative[CollocationHessianIndex(1, 0)] = m_zeroNxNuBuffer;


                if (!((m_dynamicalSystem_ptr->setControlInput(controlInputs[1])))){
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives", "Error while setting the control input.");
                    return false;
                }

                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTState(time + dT, collocationPoints[1], m_lambda, m_stateHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system state second derivative.");
                    return false;
                }

                stateSecondDerivative[CollocationHessianIndex(1, 1)] = m_stateHessianBuffer;


                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTControl(time + dT, collocationPoints[1], m_lambda, m_controlHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system control second derivative.");
                    return false;
                }

                controlSecondDerivative[CollocationHessianIndex(1, 1)] = m_controlHessianBuffer;


                if (!(m_dynamicalSystem_ptr->dynamicsSecondPartialDerivativeWRTStateControl(time + dT, collocationPoints[1], m_lambda, m_mixedHessianBuffer))) {
                    reportError(m_info.name().c_str(), "evaluateCollocationConstraintSecondDerivatives",
                                "Error while evaluating the dynamical system second derivative wrt state and control.");
                    return false;
                }

                stateControlSecondDerivative[CollocationHessianIndex(1, 1)] = m_mixedHessianBuffer;

                return true;
            }

            bool ImplicitTrapezoidal::getCollocationConstraintSecondDerivativeWRTStateSparsity(CollocationHessianSparsityMap &stateDerivativeSparsity)
            {
                if (!m_hasStateHessianSparsity) {
                    return false;
                }

                stateDerivativeSparsity = m_stateHessianSparsity;
                return true;
            }

            bool ImplicitTrapezoidal::getCollocationConstraintSecondDerivativeWRTControlSparsity(CollocationHessianSparsityMap &controlDerivativeSparsity)
            {
                if (!m_hasControlHessianSparsity) {
                    return false;
                }

                controlDerivativeSparsity = m_controlHessianSparsity;
                return true;
            }

            bool ImplicitTrapezoidal::getCollocationConstraintSecondDerivativeWRTStateControlSparsity(CollocationHessianSparsityMap &stateControlDerivativeSparsity)
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
