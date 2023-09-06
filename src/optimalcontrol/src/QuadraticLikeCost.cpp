// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/QuadraticLikeCost.h>
#include <iDynTree/Utils.h>

#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>
#include <cmath>

namespace iDynTree {
    namespace optimalcontrol {

        QuadraticLikeCost::QuadraticLikeCost(const std::string &costName)
            : Cost(costName)
            , m_timeVaryingStateHessian(nullptr)
            , m_timeVaryingStateGradient(nullptr)
            , m_timeVaryingStateCostBias(nullptr)
            , m_timeVaryingControlHessian(nullptr)
            , m_timeVaryingControlGradient(nullptr)
            , m_timeVaryingControlCostBias(nullptr)
            , m_hasSecondPartialDerivativeWRTStateSparsity(false)
            , m_hasSecondPartialDerivativeWRTControlSparsity(false)
            , m_hasSecondPartialDerivativeWRTStateControlSparsity(false)
        { }

        QuadraticLikeCost::~QuadraticLikeCost()
        { }

        bool QuadraticLikeCost::costEvaluation(double time,
                                           const iDynTree::VectorDynSize& state,
                                           const iDynTree::VectorDynSize& control,
                                           double& costValue)
        {
            double stateCost = 0, controlCost = 0;

            bool isValid = false;
            if (m_timeVaryingStateHessian) {
                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }
                stateCost += 0.5 * (toEigen(state).transpose() * toEigen(hessian) * toEigen(state))(0);
            }

            if (m_timeVaryingStateGradient) {

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (state.size() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The gradient at time: " << time << " have dimensions different from the state.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }
                stateCost += toEigen(gradient).transpose()*toEigen(state);
            }

            if (m_timeVaryingStateCostBias) {

                isValid = false;
                double stateBias = m_timeVaryingStateCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state cost bias at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                stateCost += stateBias;
            }

            if (m_timeVaryingControlHessian) {

                isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                controlCost += 0.5 * (toEigen(control).transpose() * toEigen(hessian) * toEigen(control))(0);
            }

            if (m_timeVaryingControlGradient) {

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (control.size() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time " << time << " have dimension different from the control dimension.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }
                controlCost += toEigen(gradient).transpose()*toEigen(control);
            }

            if (m_timeVaryingControlCostBias) {

                isValid = false;
                double controlBias = m_timeVaryingControlCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control cost bias at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                controlCost += controlBias;
            }

            costValue = stateCost + controlCost;
            return true;
        }

        bool QuadraticLikeCost::costFirstPartialDerivativeWRTState(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& /*control*/,
                                                               iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size());
            partialDerivative.zero();

            bool isValid = false;

            if (m_timeVaryingStateHessian) {

                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }
                toEigen(partialDerivative) += 0.5 * (toEigen(hessian) + toEigen(hessian).transpose()) * toEigen(state);
            }

            if (m_timeVaryingStateGradient) {

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (state.size() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) += toEigen(gradient);
            }

            return true;
        }

        bool QuadraticLikeCost::costFirstPartialDerivativeWRTControl(double time,
                                                                 const iDynTree::VectorDynSize& /*state*/,
                                                                 const iDynTree::VectorDynSize& control,
                                                                 iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size());
            partialDerivative.zero();
            bool isValid = false;

            if (m_timeVaryingControlHessian) {

                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }
                toEigen(partialDerivative) += 0.5 * (toEigen(hessian) + toEigen(hessian).transpose()) * toEigen(control);
            }

            if (m_timeVaryingControlGradient) {

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (control.size() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) += toEigen(gradient);
            }


            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTState(double time,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& /*control*/,
                                                                iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size(), state.size());

            if (m_timeVaryingStateHessian) {

                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = 0.5 * (toEigen(hessian) + toEigen(hessian).transpose());

            } else {

                partialDerivative.zero();

            }
            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTControl(double time,
                                                                  const iDynTree::VectorDynSize& /*state*/,
                                                                  const iDynTree::VectorDynSize& control,
                                                                  iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size(), control.size());

            if (m_timeVaryingControlHessian) {

                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = 0.5 * (toEigen(hessian) + toEigen(hessian).transpose());
            } else {
                partialDerivative.zero();
            }
            return true;
        }


        bool QuadraticLikeCost::costSecondPartialDerivativeWRTStateControl(double /*time*/,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& control,
                                                                iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size(), control.size());
            partialDerivative.zero();
            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            if (!m_hasSecondPartialDerivativeWRTStateSparsity) {
                return false;
            }
            stateSparsity = m_secondPartialDerivativeWRTStateSparsity;
            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &stateControlSparsity)
        {
            if (!m_hasSecondPartialDerivativeWRTStateControlSparsity) {
                return false;
            }
            stateControlSparsity = m_secondPartialDerivativeWRTStateControlSparsity;
            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            if (!m_hasSecondPartialDerivativeWRTControlSparsity) {
                return false;
            }
            controlSparsity = m_secondPartialDerivativeWRTControlSparsity;
            return true;
        }


    }
}
