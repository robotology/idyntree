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
            size_t numberOfMeshPoints;


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

        void MultipleShootingSolver::setNumberOfMeshPoints(size_t numberOfMeshPoints)
        {
            assert(m_pimpl);
            m_pimpl->numberOfMeshPoints = numberOfMeshPoints;
        }

        bool MultipleShootingSolver::initialize()
        {
            assert(m_pimpl);
            // 1) allocate space needed for the NLP
            // TODO: size of variable depends on the integration schema
            // size of optimisation variable
            // z = #mesh points * size of state + #mesh points * size of control

            size_t sizeOfOptimisation = 0;
            sizeOfOptimisation += m_pimpl->numberOfMeshPoints * m_pimpl->controlProblem.dynamicalSystem().lock()->stateSpaceSize();
            // TODO: for now assume Trapezoidal discretisation (no midpoint)
            sizeOfOptimisation += m_pimpl->numberOfMeshPoints * m_pimpl->controlProblem.dynamicalSystem().lock()->controlSpaceSize();
            m_pimpl->optimisationVariable.resize(sizeOfOptimisation);

            // do the same for the constraints
            size_t numberOfConstraints = 0;
            // x_i+1 - x_i - h_i/2 * (f_i - f_i+1)
            numberOfConstraints += m_pimpl->numberOfMeshPoints; // dynamic equations => defect constraints
            // also the path constraints must be discretized

            // do the same for the cost





            return true;
        }

        bool MultipleShootingSolver::solve()
        {
            assert(m_pimpl);
            return false;
        }
    }
}
