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

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H

#include <cstddef>

namespace iDynTree {

    class VectorDynSize;


    namespace optimalcontrol {

        class OptimalControlProblem;

        class OptimalControlSolver
        {

        public:
            OptimalControlSolver() = delete;

            OptimalControlSolver(OptimalControlProblem&);
            virtual ~OptimalControlSolver();

            virtual bool initialize() = 0;
            virtual bool solve() = 0;

        protected:
//            class OptimalControlSolverPimpl;
//            OptimalControlSolverPimpl* m_pimpl;

        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLSOLVER_H */
