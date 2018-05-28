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
