// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimalControlSolver.h>

#include <iDynTree/OptimalControlProblem.h>

#include <iDynTree/VectorDynSize.h>

#include <cassert>

namespace iDynTree {
    namespace optimalcontrol {

        OptimalControlSolver::OptimalControlSolver(const std::shared_ptr<OptimalControlProblem> ocProblem) {}
        OptimalControlSolver::~OptimalControlSolver() {}
        
        // MARK: Private implementation
//        class OptimalControlSolver::OptimalControlSolverPimpl
//        {
//        public:
//            OptimalControlSolverPimpl(OptimalControlProblem& problem)
//            : controlProblem(problem) {}
//
//            OptimalControlProblem& controlProblem;
//
//            iDynTree::VectorDynSize lastSolution;
//
//            size_t numberOfMeshPoints;
//
//        };
//
//        // MARK: Class implementation
//        OptimalControlSolver::OptimalControlSolver(OptimalControlProblem& problem)
//        : m_pimpl(new OptimalControlSolverPimpl(problem))
//        {
//            assert(m_pimpl);
//        }
//
//
//        const iDynTree::VectorDynSize& OptimalControlSolver::lastSolution()
//        {
//            assert(m_pimpl);
//            return m_pimpl->lastSolution;
//        }
//
//        void OptimalControlSolver::setInitialGuess(const iDynTree::VectorDynSize &initialGuess)
//        {
//            assert(m_pimpl);
//            m_pimpl->lastSolution = initialGuess;
//        }
//
//        void OptimalControlSolver::setNumberOfMeshPoints(size_t numberOfMeshPoints)
//        {
//            assert(m_pimpl);
//            m_pimpl->numberOfMeshPoints = numberOfMeshPoints;
//        }


    }
}
