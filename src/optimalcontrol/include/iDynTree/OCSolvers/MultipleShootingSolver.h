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

#ifndef IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H

#include "iDynTree/OptimalControlSolver.h"

namespace iDynTree {
    namespace optimalcontrol {

        class OptimalControlProblem;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class MultipleShootingSolver
        : public OptimalControlSolver {

        public:
            MultipleShootingSolver(OptimalControlProblem&);

            // FIXME: These two cannot be used as VectorDynTree
            // as they are trajectories, not single vectors
            void setInitialGuess(const iDynTree::VectorDynSize& initialGuess);
            const iDynTree::VectorDynSize& lastSolution();


            virtual bool initialize() override;
            virtual bool solve() override;

        private:

            class MultipleShootingSolverPimpl;
            MultipleShootingSolverPimpl* m_pimpl;


        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H */
