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

#include "iDynTree/QuadraticCost.h"

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
