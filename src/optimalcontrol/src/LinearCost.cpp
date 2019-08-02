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

#include <iDynTree/LinearCost.h>
#include <iDynTree/Core/Utils.h>

namespace iDynTree {
    namespace optimalcontrol {

        LinearCost::LinearCost(const std::string &costName)
            : QuadraticLikeCost(costName)
        {
            m_hasSecondPartialDerivativeWRTStateSparsity = true;
            m_hasSecondPartialDerivativeWRTControlSparsity = true;
            m_hasSecondPartialDerivativeWRTStateControlSparsity = true;
        }

        LinearCost::~LinearCost()
        { }

        bool LinearCost::setStateCost(const VectorDynSize &stateGradient)
        {
            m_timeVaryingStateGradient.reset(new TimeInvariantVector(stateGradient));

            return true;
        }

        bool LinearCost::setStateCost(std::shared_ptr<TimeVaryingVector> timeVaryingStateGradient)
        {
            if (!timeVaryingStateGradient) {
                reportError("LinearCost", "setStateCost", "Empty gradient pointer.");
                return false;
            }

            m_timeVaryingStateGradient = timeVaryingStateGradient;

            return true;
        }

        bool LinearCost::setControlCost(const VectorDynSize &controlGradient)
        {

            m_timeVaryingControlGradient.reset(new TimeInvariantVector(controlGradient));

            return true;
        }

        bool LinearCost::setControlCost(std::shared_ptr<TimeVaryingVector> timeVaryingControlGradient)
        {

            if (!timeVaryingControlGradient) {
                reportError("QuadraticCost", "setControlCost", "Empty gradient pointer.");
                return false;
            }

            m_timeVaryingControlGradient = timeVaryingControlGradient;

            return true;
        }

        bool LinearCost::setCostBias(double stateCostBias, double controlCostBias)
        {
            m_timeVaryingStateCostBias.reset(new TimeInvariantDouble(stateCostBias));
            m_timeVaryingControlCostBias.reset(new TimeInvariantDouble(controlCostBias));
            return true;
        }

        bool LinearCost::setCostBias(std::shared_ptr<TimeVaryingDouble> timeVaryingStateCostBias, std::shared_ptr<TimeVaryingDouble> timeVaryingControlCostBias)
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

    }
}
