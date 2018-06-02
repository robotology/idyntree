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

#include <iDynTree/QuadraticCost.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <cmath>

namespace iDynTree {
    namespace optimalcontrol {

        QuadraticCost::QuadraticCost(const std::string &costName)
            : Cost(costName)
            , m_timeVaryingStateHessian(nullptr)
            , m_timeVaryingStateGradient(nullptr)
            , m_timeVaryingControlHessian(nullptr)
            , m_timeVaryingControlGradient(nullptr)
            , m_stateCostScale(0.0)
            , m_controlCostScale(0.0)
        {
            m_timeVaryingStateCostBias = std::make_shared<TimeInvariantDouble>(0.0);
            m_timeVaryingControlCostBias = std::make_shared<TimeInvariantDouble>(0.0);
        }

        QuadraticCost::~QuadraticCost()
        { }

        bool QuadraticCost::setStateCost(const MatrixDynSize &stateHessian, const VectorDynSize &stateGradient)
        {
            if (stateHessian.rows() != stateHessian.cols()) {
                reportError("QuadraticCost", "setStateCost", "The stateHessian matrix is supposed to be square.");
                return false;
            }

            if (stateHessian.rows() != stateGradient.size()) {
                reportError("QuadraticCost", "setStateCost", "stateHessian and stateGradient have inconsistent dimensions.");
                return false;
            }

            m_timeVaryingStateHessian.reset(new TimeInvariantMatrix(stateHessian));
            m_timeVaryingStateGradient.reset(new TimeInvariantVector(stateGradient));
            m_stateCostScale = 1.0;

            return true;
        }

        bool QuadraticCost::setStateCost(std::shared_ptr<TimeVaryingMatrix> timeVaryingStateHessian, std::shared_ptr<TimeVaryingVector> timeVaryingStateGradient)
        {
            if (!timeVaryingStateHessian) {
                reportError("QuadraticCost", "setStateCost", "Empty hessian pointer.");
                return false;
            }

            if (!timeVaryingStateGradient) {
                reportError("QuadraticCost", "setStateCost", "Empty gradient pointer.");
                return false;
            }

            m_timeVaryingStateHessian = timeVaryingStateHessian;
            m_timeVaryingStateGradient = timeVaryingStateGradient;

            return true;
        }

        bool QuadraticCost::setControlCost(const MatrixDynSize &controlHessian, const VectorDynSize &controlGradient)
        {
            if (controlHessian.rows() != controlHessian.cols()) {
                reportError("QuadraticCost", "setControlCost", "The controlHessian matrix is supposed to be square.");
                return false;
            }

            if (controlHessian.rows() != controlGradient.size()) {
                reportError("QuadraticCost", "setControlCost", "controlHessian and controlGradient have inconsistent dimensions.");
                return false;
            }

            m_timeVaryingControlHessian.reset(new TimeInvariantMatrix(controlHessian));
            m_timeVaryingControlGradient.reset(new TimeInvariantVector(controlGradient));
            m_controlCostScale = 1.0;

            return true;
        }

        bool QuadraticCost::setControlCost(std::shared_ptr<TimeVaryingMatrix> timeVaryingControlHessian, std::shared_ptr<TimeVaryingVector> timeVaryingControlGradient)
        {
            if (!timeVaryingControlHessian) {
                reportError("QuadraticCost", "setControlCost", "Empty hessian pointer.");
                return false;
            }

            if (!timeVaryingControlGradient) {
                reportError("QuadraticCost", "setControlCost", "Empty gradient pointer.");
                return false;
            }

            m_timeVaryingControlHessian = timeVaryingControlHessian;
            m_timeVaryingControlGradient = timeVaryingControlGradient;

            return true;
        }

        bool QuadraticCost::setCostBias(double stateCostBias, double controlCostBias)
        {
            m_timeVaryingStateCostBias.reset(new TimeInvariantDouble(stateCostBias));
            m_timeVaryingControlCostBias.reset(new TimeInvariantDouble(controlCostBias));
            return true;
        }

        bool QuadraticCost::setCostBias(std::shared_ptr<TimeVaryingDouble> timeVaryingStateCostBias,
                                        std::shared_ptr<TimeVaryingDouble> timeVaryingControlCostBias)
        {
            if (!timeVaryingStateCostBias) {
                reportError("QuadraticCost", "addCostBias", "The timeVaryingStateCostBias pointer is empty.");
                return false;
            }

            if (!timeVaryingControlCostBias) {
                reportError("QuadraticCost", "addCostBias", "The timeVaryingControlCostBias pointer is empty.");
                return false;
            }

            m_timeVaryingStateCostBias = timeVaryingStateCostBias;
            m_timeVaryingControlCostBias = timeVaryingControlCostBias;
            return true;
        }

        bool QuadraticCost::costEvaluation(double time,
                                           const iDynTree::VectorDynSize& state,
                                           const iDynTree::VectorDynSize& control,
                                           double& costValue)
        {
            double stateCost = 0, controlCost = 0;

            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                double stateBias = m_timeVaryingStateCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state cost bias at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                stateCost = m_stateCostScale * 0.5 * (toEigen(state).transpose() * toEigen(hessian) * toEigen(state))(0) + toEigen(gradient).transpose()*toEigen(state) + stateBias;
            }

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                double controlBias = m_timeVaryingControlCostBias->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control cost bias at time: " << time << ".";
                    reportError("QuadraticCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                controlCost = m_controlCostScale * 0.5 * (toEigen(control).transpose() * toEigen(hessian) * toEigen(control))(0) + toEigen(gradient).transpose()*toEigen(control) + controlBias;
            }

            costValue = stateCost + controlCost;
            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTState(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& /*control*/,
                                                               iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size());

            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingStateGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state gradient at time: " << time << ".";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = m_stateCostScale * toEigen(hessian) * toEigen(state) + toEigen(gradient);
            } else {
                partialDerivative.zero();
            }

            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTControl(double time,
                                                                 const iDynTree::VectorDynSize& /*state*/,
                                                                 const iDynTree::VectorDynSize& control,
                                                                 iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size());

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                isValid = false;
                const VectorDynSize &gradient = m_timeVaryingControlGradient->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control gradient at time: " << time << ".";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != gradient.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian and the gradient at time: " << time << " have different dimensions.";
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = m_controlCostScale * toEigen(hessian) * toEigen(control) + toEigen(gradient);
            } else {
                partialDerivative.zero();
            }
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTState(double time,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& /*control*/,
                                                                iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size(), state.size());
            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingStateHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the state dimension.";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = m_stateCostScale * toEigen(hessian);
            } else {
                partialDerivative.zero();
            }
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTControl(double time,
                                                                  const iDynTree::VectorDynSize& /*state*/,
                                                                  const iDynTree::VectorDynSize& control,
                                                                  iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size(), control.size());

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                bool isValid = false;
                const MatrixDynSize &hessian = m_timeVaryingControlHessian->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control hessian at time: " << time << ".";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != hessian.cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control hessian at time: " << time << " is not squared.";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The hessian at time: " << time << " does not match the control dimension.";
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                toEigen(partialDerivative) = m_controlCostScale * toEigen(hessian);
            } else {
                partialDerivative.zero();
            }
            return true;
        }


        bool QuadraticCost::costSecondPartialDerivativeWRTStateControl(double /*time*/,
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
