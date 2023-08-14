// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_LINEARCOST_H
#define IDYNTREE_OPTIMALCONTROL_LINEARCOST_H

#include <iDynTree/QuadraticLikeCost.h>
#include <iDynTree/TimeVaryingObject.h>


namespace iDynTree {
    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class LinearCost : public QuadraticLikeCost {

        public:
            LinearCost(const std::string& costName);

            ~LinearCost();

            bool setStateCost(const iDynTree::VectorDynSize& stateGradient);

            bool setStateCost(std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> timeVaryingStateGradient);

            bool setControlCost(const iDynTree::VectorDynSize& controlGradient);

            bool setControlCost(std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> timeVaryingControlGradient);

            bool setCostBias(double stateCostBias, double controlCostBias);

            bool setCostBias(std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingDouble> timeVaryingStateCostBias,
                             std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingDouble> timeVaryingControlCostBias);

        };


    }
}

#endif // IDYNTREE_OPTIMALCONTROL_LINEARCOST_H
