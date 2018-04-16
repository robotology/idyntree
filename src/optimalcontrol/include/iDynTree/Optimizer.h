/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMIZER_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMIZER_H

#include <memory>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/OptimizationProblem.h>

namespace iDynTree {

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class Optimizer {
        protected:

            std::shared_ptr<OptimizationProblem> m_problem;

        public:

            Optimizer();

            virtual ~Optimizer();

            virtual bool setProblem(std::shared_ptr<OptimizationProblem> problem);

            virtual const std::weak_ptr<OptimizationProblem> problem() const;

            virtual bool solve() = 0; //warm start capabilities should be implemented in the solver specific interface

            virtual bool getPrimalVariables(VectorDynSize &primalVariables);

            virtual bool getDualVariables(VectorDynSize &dualVariables);

            virtual bool getOptimalCost(double &optimalCost);

            virtual double minusInfinity();

            virtual double plusInfinity();
        };

    }

}

#endif // IDYNTREE_OPTIMALCONTROL_OPTIMIZER_H
