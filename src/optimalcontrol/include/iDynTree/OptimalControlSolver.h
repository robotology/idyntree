// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H

#include <memory>
#include <cstddef>

namespace iDynTree {

    class VectorDynSize;


    namespace optimalcontrol {

        class OptimalControlProblem;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class OptimalControlSolver
        {

        public:
            OptimalControlSolver() = delete;

            OptimalControlSolver(const std::shared_ptr<OptimalControlProblem> ocProblem);

            virtual ~OptimalControlSolver();

            virtual bool solve() = 0;

        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H */
