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

#include "iDynTree/OCSolvers/MultipleShootingSolver.h"

#include "iDynTree/OptimalControlProblem.h"
#include "iDynTree/DynamicalSystem.h"

#include <iDynTree/Core/VectorDynSize.h>

#include <cassert>

// List of TODOs
// TODO: varying time interval
// TODO: decide later how to structure the hierarchy
// TODO: understand if some parameters should go in the Integrator class or not

namespace iDynTree {
    namespace optimalcontrol
    {
        // MARK: Private implementation
        class MultipleShootingSolver::MultipleShootingSolverPimpl {
        public:
            MultipleShootingSolverPimpl(OptimalControlProblem& problem)
            : controlProblem(problem) {}

            OptimalControlProblem& controlProblem;
            iDynTree::VectorDynSize lastSolution;

            iDynTree::VectorDynSize optimisationVariable;



        };

        // MARK: Class implementation

        MultipleShootingSolver::MultipleShootingSolver(OptimalControlProblem& problem)
        : OptimalControlSolver(problem)
        , m_pimpl(new MultipleShootingSolverPimpl(problem))
        {
            assert(m_pimpl);
        }

        void MultipleShootingSolver::setInitialGuess(const iDynTree::VectorDynSize& initialGuess)
        {
            assert(m_pimpl);
            m_pimpl->lastSolution = initialGuess;
        }

        const iDynTree::VectorDynSize& MultipleShootingSolver::lastSolution()
        {
            assert(m_pimpl);
            return m_pimpl->lastSolution;
        }

        bool MultipleShootingSolver::initialize()
        {
            assert(m_pimpl);

            return true;
        }

        bool MultipleShootingSolver::solve()
        {
            assert(m_pimpl);
            return false;
        }
    }
}
