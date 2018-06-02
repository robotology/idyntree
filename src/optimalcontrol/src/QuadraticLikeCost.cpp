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

#include <iDynTree/QuadraticLikeCost.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <cmath>

namespace iDynTree {
    namespace optimalcontrol {

        QuadraticLikeCost::QuadraticLikeCost(const std::string &costName)
            : Cost(costName)
            , m_timeVaryingStateHessian(nullptr)
            , m_timeVaryingStateGradient(nullptr)
            , m_timeVaryingControlHessian(nullptr)
            , m_timeVaryingControlGradient(nullptr)
            , m_costsState(false)
            , m_costsControl(false)
        {
            m_timeVaryingStateCostBias = std::make_shared<TimeInvariantDouble>(0.0);
            m_timeVaryingControlCostBias = std::make_shared<TimeInvariantDouble>(0.0);
        }

        QuadraticLikeCost::~QuadraticLikeCost()
        { }

        bool QuadraticLikeCost::costEvaluation(double time,
                                           const iDynTree::VectorDynSize& state,
                                           const iDynTree::VectorDynSize& control,
                                           double& costValue)
        {
            double stateCost = 0, controlCost = 0;

            if (m_costsState){
                bool isValid = false;
                if (!m_timeVaryingStateHessian) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The state cost is activated but the timeVaryingStateHessian pointer is empty.");
                    return false;
                }
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

                if (!m_timeVaryingStateGradient) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The state cost is activated but the timeVaryingStateGradient pointer is empty.");
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (!m_timeVaryingStateCostBias) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The state cost is activated but the timeVaryingStateCostBias pointer is empty.");
                    return false;
                }
                isValid = false;
                double stateBias = m_timeVaryingStateCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state cost bias at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                stateCost = 0.5 * (toEigen(state).transpose() * toEigen(hessian) * toEigen(state))(0) + toEigen(gradient).transpose()*toEigen(state) + stateBias;
            }

            if (m_costsControl){
                if (!m_timeVaryingControlHessian) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The control cost is activated but the timeVaryingControlHessian pointer is empty.");
                    return false;
                }

                bool isValid = false;
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

                if (!m_timeVaryingControlGradient) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The control cost is activated but the timeVaryingControlGradient pointer is empty.");
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (!m_timeVaryingControlCostBias) {
                    reportError("QuadraticLikeCost", "costEvaluation", "The control cost is activated but the timeVaryingControlCostBias pointer is empty.");
                    return false;
                }

                isValid = false;
                double controlBias = m_timeVaryingControlCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control cost bias at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                controlCost = 0.5 * (toEigen(control).transpose() * toEigen(hessian) * toEigen(control))(0) + toEigen(gradient).transpose()*toEigen(control) + controlBias;
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

            if (m_costsState){
                if (!m_timeVaryingStateHessian) {
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", "The state cost is activated but the timeVaryingStateHessian pointer is empty.");
                    return false;
                }

                bool isValid = false;
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

                if (!m_timeVaryingStateGradient) {
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", "The state cost is activated but the timeVaryingStateGradient pointer is empty.");
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = toEigen(hessian) * toEigen(state) + toEigen(gradient);
            } else {
                partialDerivative.zero();
            }

            return true;
        }

        bool QuadraticLikeCost::costFirstPartialDerivativeWRTControl(double time,
                                                                 const iDynTree::VectorDynSize& /*state*/,
                                                                 const iDynTree::VectorDynSize& control,
                                                                 iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size());

            if (m_costsControl){
                if (!m_timeVaryingControlHessian) {
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", "The control cost is activated but the timeVaryingControlHessian pointer is empty.");
                    return false;
                }

                bool isValid = false;
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

                if (!m_timeVaryingControlGradient) {
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", "The control cost is activated but the timeVaryingControlGradient pointer is empty.");
                    return false;
                }
                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticLikeCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = toEigen(hessian) * toEigen(control) + toEigen(gradient);
            } else {
                partialDerivative.zero();
            }
            return true;
        }

        bool QuadraticLikeCost::costSecondPartialDerivativeWRTState(double time,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& /*control*/,
                                                                iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size(), state.size());
            if (m_costsState){
                if (!m_timeVaryingStateHessian) {
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTState", "The state cost is activated but the timeVaryingStateHessian pointer is empty.");
                    return false;
                }
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

                partialDerivative = hessian;
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

            if (m_costsControl){
                if (!m_timeVaryingControlHessian) {
                    reportError("QuadraticLikeCost", "costSecondPartialDerivativeWRTControl", "The control cost is activated but the timeVaryingControlHessian pointer is empty.");
                    return false;
                }
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

                partialDerivative = hessian;
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


    }
}
