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
            , m_stateCostScale(0.0)
            , m_controlCostScale(0.0)
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

            m_stateHessian = stateHessian;
            m_stateGradient = stateGradient;
            m_stateCostScale = 1.0;

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

            m_controlHessian = controlHessian;
            m_controlGradient = controlGradient;
            m_controlCostScale = 1.0;

            return true;
        }

        bool QuadraticCost::costEvaluation(double /*time*/,
                                           const iDynTree::VectorDynSize& state,
                                           const iDynTree::VectorDynSize& control,
                                           double& costValue)
        {
            double stateCost = 0, controlCost = 0;

            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                if (m_stateHessian.rows() != state.size()) {
                    reportError("QuadraticCost", "costEvaluation", "The specified state hessian matrix dimensions do not match the state dimension.");
                    return false;
                }
                stateCost = m_stateCostScale * 0.5 * (toEigen(state).transpose() * toEigen(m_stateHessian) * toEigen(state))(0) + toEigen(m_stateGradient).transpose()*toEigen(state);
            }

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                if (m_controlHessian.rows() != control.size()) {
                    reportError("QuadraticCost", "costEvaluation", "The specified control hessian matrix dimensions do not match the control dimension.");
                    return false;
                }
                controlCost = m_controlCostScale * 0.5 * (toEigen(control).transpose() * toEigen(m_controlHessian) * toEigen(control))(0) + toEigen(m_controlGradient).transpose()*toEigen(control);
            }

            costValue = stateCost + controlCost;
            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTState(double /*time*/,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& /*control*/,
                                                               iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size());

            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                if (m_stateHessian.rows() != state.size()) {
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTState", "The specified state hessian matrix dimensions do not match the state dimension.");
                    return false;
                }
                toEigen(partialDerivative) = m_stateCostScale * toEigen(m_stateHessian) * toEigen(state) + toEigen(m_stateGradient);
            } else {
                partialDerivative.zero();
            }

            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTControl(double /*time*/,
                                                          const iDynTree::VectorDynSize& /*state*/,
                                                          const iDynTree::VectorDynSize& control,
                                                          iDynTree::VectorDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size());

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                if (m_controlHessian.rows() != control.size()) {
                    reportError("QuadraticCost", "costFirstPartialDerivativeWRTControl", "The specified control hessian matrix dimensions do not match the control dimension.");
                    return false;
                }
                toEigen(partialDerivative) = m_controlCostScale * toEigen(m_controlHessian) * toEigen(control) + toEigen(m_controlGradient);
            } else {
                partialDerivative.zero();
            }
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTState(double /*time*/,
                                                         const iDynTree::VectorDynSize& state,
                                                         const iDynTree::VectorDynSize& /*control*/,
                                                         iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(state.size(), state.size());
            if (!checkDoublesAreEqual(m_stateCostScale, 0, 1E-30)){
                if (m_stateHessian.rows() != state.size()) {
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTState", "The specified state hessian matrix dimensions do not match the state dimension.");
                    return false;
                }
                toEigen(partialDerivative) = m_stateCostScale * toEigen(m_stateHessian);
            } else {
                partialDerivative.zero();
            }
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTControl(double /*time*/,
                                                           const iDynTree::VectorDynSize& /*state*/,
                                                           const iDynTree::VectorDynSize& control,
                                                           iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative.resize(control.size(), control.size());

            if (!checkDoublesAreEqual(m_controlCostScale, 0, 1E-30)){
                if (m_controlHessian.rows() != control.size()) {
                    reportError("QuadraticCost", "costSecondPartialDerivativeWRTControl", "The specified control hessian matrix dimensions do not match the control dimension.");
                    return false;
                }
                toEigen(partialDerivative) = m_controlCostScale * toEigen(m_controlHessian);
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
