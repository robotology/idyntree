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

#include <iDynTree/LinearConstraint.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>

namespace iDynTree {
    namespace optimalcontrol {

        LinearConstraint::~LinearConstraint() {}

        bool LinearConstraint::evaluateConstraint(double time,
                                                  const VectorDynSize& state,
                                                  const VectorDynSize& control,
                                                  VectorDynSize& constraint)
        {

             iDynTree::iDynTreeEigenMatrixMap stateConstraint = iDynTree::toEigen(m_stateConstraintMatrix);
             iDynTree::iDynTreeEigenMatrixMap controlConstraint = iDynTree::toEigen(m_controlConstraintMatrix);

            assert(stateConstraint.rows() + controlConstraint.rows() == constraintSize());

            iDynTree::toEigen(constraint).head(stateConstraint.rows()) = stateConstraint * iDynTree::toEigen(state);
            iDynTree::toEigen(constraint).tail(controlConstraint.rows()) = controlConstraint * iDynTree::toEigen(control);


            return true;

        }

        bool LinearConstraint::constraintJacobianWRTState(double time,
                                                const VectorDynSize& state,
                                                const VectorDynSize& control,
                                                MatrixDynSize& jacobian)
        {
            iDynTree::toEigen(jacobian) = iDynTree::toEigen(m_stateConstraintMatrix);
            return true;
        }

        bool LinearConstraint::constraintJacobianWRTControl(double time,
                                                  const VectorDynSize& state,
                                                  const VectorDynSize& control,
                                                  MatrixDynSize& jacobian)
        {
            iDynTree::toEigen(jacobian) = iDynTree::toEigen(m_controlConstraintMatrix);
            return true;
        }

    }
}
