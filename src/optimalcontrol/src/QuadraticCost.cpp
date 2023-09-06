// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/QuadraticCost.h>
#include <iDynTree/Utils.h>

#include <iDynTree/EigenHelpers.h>
#include <cmath>

namespace iDynTree {
    namespace optimalcontrol {

        QuadraticCost::QuadraticCost(const std::string &costName)
            : QuadraticLikeCost(costName)
        {
            m_hasSecondPartialDerivativeWRTStateControlSparsity = true;
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

        bool iDynTree::optimalcontrol::QuadraticCost::setStateHessianSparsity(const SparsityStructure &stateSparsity)
        {
            if (!stateSparsity.isValid()) {
                reportError("QuadraticCost", "setStateHessianSparsity", "The sparsity vectors have different number of entries.");
                return false;
            }
            m_hasSecondPartialDerivativeWRTStateSparsity = true;
            m_secondPartialDerivativeWRTStateSparsity = stateSparsity;
            return true;
        }

        bool QuadraticCost::setControlHessianSparsity(const SparsityStructure &controlSparsity)
        {
            if (!controlSparsity.isValid()) {
                reportError("QuadraticCost", "setControlHessianSparsity", "The sparsity vectors have different number of entries.");
                return false;
            }
            m_hasSecondPartialDerivativeWRTControlSparsity = true;
            m_secondPartialDerivativeWRTControlSparsity = controlSparsity;
            return true;
        }

    }
}
