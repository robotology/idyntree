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

        LinearConstraint::LinearConstraint(size_t size, const std::string name)
        : Constraint(size, name)
        , m_constrainsState(false)
        , m_constrainsControl(false)
        , m_stateConstraintsBuffer(static_cast<unsigned int>(size))
        , m_controlConstraintsBuffer(static_cast<unsigned int>(size))
        {
            m_stateConstraintsBuffer.zero();
            m_controlConstraintsBuffer.zero();
        }

        LinearConstraint::~LinearConstraint() {}

        bool LinearConstraint::setStateConstraintMatrix(const MatrixDynSize &constraintMatrix)
        {
            if (constraintMatrix.rows() != constraintSize()) {
                reportError("LinearConstraint", "setStateConstraintMatrix", "The number of rows of the constraintMatrix is different from the specified constraint size.");
                return false;
            }

            m_stateConstraintMatrix = constraintMatrix;
            m_constrainsState = true;

            return true;
        }

        bool LinearConstraint::setControlConstraintMatrix(const MatrixDynSize &constraintMatrix)
        {
            if (constraintMatrix.rows() != constraintSize()) {
                reportError("LinearConstraint", "setControlConstraintMatrix", "The number of rows of the constraintMatrix is different from the specified constraint size.");
                return false;
            }

            m_controlConstraintMatrix = constraintMatrix;
            m_constrainsControl = true;

            return true;
        }


        bool LinearConstraint::evaluateConstraint(double time,
                                                  const VectorDynSize& state,
                                                  const VectorDynSize& control,
                                                  VectorDynSize& constraint)
        {
            if (m_constrainsState) {
                if (m_stateConstraintMatrix.cols() != state.size()) {
                    reportError("LinearConstraint", "evaluateConstraint", "The dimension of the specified constraint matrix and the state do not match.");
                    return false;
                }
                iDynTree::iDynTreeEigenMatrixMap stateConstraint = iDynTree::toEigen(m_stateConstraintMatrix);

                iDynTree::toEigen(m_stateConstraintsBuffer) = stateConstraint * iDynTree::toEigen(state);
            }


            if (m_constrainsControl) {
                if (m_controlConstraintMatrix.cols() != control.size()) {
                    reportError("LinearConstraint", "evaluateConstraint", "The dimension of the specified constraint matrix and the control do not match.");
                    return false;
                }

                iDynTree::iDynTreeEigenMatrixMap controlConstraint = iDynTree::toEigen(m_controlConstraintMatrix);
                iDynTree::toEigen(m_controlConstraintsBuffer) = controlConstraint * iDynTree::toEigen(control);
            }

            constraint.resize(static_cast<unsigned int>(constraintSize()));
            toEigen(constraint) = iDynTree::toEigen(m_stateConstraintsBuffer) + iDynTree::toEigen(m_controlConstraintsBuffer); //the buffers are zero if not constrained

            return true;

        }

        bool LinearConstraint::constraintJacobianWRTState(double /*time*/,
                                                const VectorDynSize& state,
                                                const VectorDynSize& /*control*/,
                                                MatrixDynSize& jacobian)
        {
            if (m_constrainsState) {
                if (m_stateConstraintMatrix.cols() != state.size()) {
                    reportError("LinearConstraint", "constraintJacobianWRTState", "The dimension of the specified constraint matrix and the state do not match.");
                    return false;
                }
            } else {
                if ((m_stateConstraintMatrix.rows() != static_cast<unsigned int>(constraintSize())) ||
                        (m_stateConstraintMatrix.cols() != state.size())) {
                    m_stateConstraintMatrix.resize(static_cast<unsigned int>(constraintSize()), state.size());
                    m_stateConstraintMatrix.zero();
                }
            }
            jacobian = m_stateConstraintMatrix;

            return true;
        }

        bool LinearConstraint::constraintJacobianWRTControl(double /*time*/,
                                                  const VectorDynSize& /*state*/,
                                                  const VectorDynSize& control,
                                                  MatrixDynSize& jacobian)
        {
            if (m_constrainsControl) {
                if (m_controlConstraintMatrix.cols() != control.size()) {
                    reportError("LinearConstraint", "constraintJacobianWRTControl", "The dimension of the specified constraint matrix and the control do not match.");
                    return false;
                }
            } else {
                if ((m_controlConstraintMatrix.rows() != static_cast<unsigned int>(constraintSize())) ||
                        (m_controlConstraintMatrix.cols() != control.size())) {
                    m_controlConstraintMatrix.resize(static_cast<unsigned int>(constraintSize()), control.size());
                    m_controlConstraintMatrix.zero();
                }
            }
            jacobian = m_controlConstraintMatrix;
            return true;
        }

    }
}
