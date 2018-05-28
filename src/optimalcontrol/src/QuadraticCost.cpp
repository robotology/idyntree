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

#include <iDynTree/QuadraticCost.h>

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {
    namespace optimalcontrol {

        QuadraticCost::QuadraticCost(const iDynTree::MatrixDynSize& Q,
                                     const iDynTree::MatrixDynSize& R, const std::string &costName)
        : Cost(costName)
        , m_stateCostMatrix(Q)
        , m_controlCostMatrix(R)
        , m_stateControlCostDerivativeMatrix(Q.rows(), R.rows())
        {
            m_stateControlCostDerivativeMatrix.zero();
        }

        bool QuadraticCost::costEvaluation(double /*time*/,
                                           const iDynTree::VectorDynSize& state,
                                           const iDynTree::VectorDynSize& control,
                                           double& costValue)
        {
            costValue = 0.5 * (iDynTree::toEigen(state).transpose() * iDynTree::toEigen(m_stateCostMatrix) * iDynTree::toEigen(state)
                          + iDynTree::toEigen(control).transpose() * iDynTree::toEigen(m_controlCostMatrix) * iDynTree::toEigen(control))(0);
            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTState(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& control,
                                                               iDynTree::VectorDynSize& partialDerivative)
        {
            // TODO: to be checked with the formalism needed: A x or x^T A ?
            iDynTree::toEigen(partialDerivative) = iDynTree::toEigen(m_stateCostMatrix) * iDynTree::toEigen(state);
            return true;
        }

        bool QuadraticCost::costFirstPartialDerivativeWRTControl(double time,
                                                          const iDynTree::VectorDynSize& state,
                                                          const iDynTree::VectorDynSize& control,
                                                          iDynTree::VectorDynSize& partialDerivative)
        {
            // TODO: to be checked with the formalism needed: A x or x^T A ?
            iDynTree::toEigen(partialDerivative) = iDynTree::toEigen(m_controlCostMatrix) * iDynTree::toEigen(control);
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTState(double time,
                                                         const iDynTree::VectorDynSize& state,
                                                         const iDynTree::VectorDynSize& control,
                                                         iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative = m_stateCostMatrix;
            return true;
        }

        bool QuadraticCost::costSecondPartialDerivativeWRTControl(double time,
                                                           const iDynTree::VectorDynSize& state,
                                                           const iDynTree::VectorDynSize& control,
                                                           iDynTree::MatrixDynSize& partialDerivative)
        {
            partialDerivative = m_controlCostMatrix;
            return true;
        }


        bool QuadraticCost::costSecondPartialDerivativeWRTStateControl(double time,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& control,
                                                                iDynTree::MatrixDynSize& partialDerivative)
        {
            // return empty matrix
            // or simply zero the matrix??
            partialDerivative = m_stateControlCostDerivativeMatrix;
            return true;
        }


    }
}
