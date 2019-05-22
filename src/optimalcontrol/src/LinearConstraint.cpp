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

        class LinearConstraint::LinearConstraintImplementation {
        public:
            bool constrainsState, constrainsControl;
            std::shared_ptr<TimeVaryingMatrix> stateConstraintMatrix;
            std::shared_ptr<TimeVaryingMatrix> controlConstraintMatrix;
            iDynTree::VectorDynSize stateConstraintsBuffer, controlConstraintsBuffer;
            bool hasStateSparsity, hasControlSparsity;
            SparsityStructure stateSparsity, controlSparsity;

            LinearConstraintImplementation(size_t size)
                : constrainsState(false)
                , constrainsControl(false)
                , stateConstraintsBuffer(static_cast<unsigned int>(size))
                , controlConstraintsBuffer(static_cast<unsigned int>(size))
                , hasStateSparsity(false)
                , hasControlSparsity(false)
            {
                stateConstraintsBuffer.zero();
                controlConstraintsBuffer.zero();
            }
        };

        LinearConstraint::LinearConstraint(size_t size, const std::string name)
        : Constraint(size, name)
        , m_pimpl(new LinearConstraintImplementation(size))
        { }

        LinearConstraint::LinearConstraint(size_t size, const std::string name, const SparsityStructure &stateSparsity, const SparsityStructure &controlSparsity)
            : Constraint(size, name)
            , m_pimpl(new LinearConstraintImplementation(size))
        {
            if (stateSparsity.isValid()) {
                m_pimpl->hasStateSparsity = true;
                m_pimpl->stateSparsity = stateSparsity;
            }
            if (controlSparsity.isValid()) {
                m_pimpl->hasControlSparsity = true;
                m_pimpl->controlSparsity = controlSparsity;
            }
        }

        LinearConstraint::~LinearConstraint()
        {
            if (m_pimpl) {
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool LinearConstraint::setStateConstraintMatrix(const MatrixDynSize &constraintMatrix)
        {
            if (constraintMatrix.rows() != constraintSize()) {
                reportError(name().c_str(), "setStateConstraintMatrix", "The number of rows of the constraintMatrix is different from the specified constraint size.");
                return false;
            }

            m_pimpl->stateConstraintMatrix = std::make_shared<TimeInvariantMatrix>(constraintMatrix);
            m_pimpl->constrainsState = true;

            return true;
        }

        bool LinearConstraint::setControlConstraintMatrix(const MatrixDynSize &constraintMatrix)
        {
            if (constraintMatrix.rows() != constraintSize()) {
                reportError(name().c_str(), "setControlConstraintMatrix", "The number of rows of the constraintMatrix is different from the specified constraint size.");
                return false;
            }

            m_pimpl->controlConstraintMatrix = std::make_shared<TimeInvariantMatrix>(constraintMatrix);
            m_pimpl->constrainsControl = true;

            return true;
        }

        bool LinearConstraint::setStateConstraintMatrix(std::shared_ptr<TimeVaryingMatrix> constraintMatrix)
        {
            if (!constraintMatrix) {
                reportError(name().c_str(), "setStateConstraintMatrix", "Empty constraintMatrix pointer.");
                return false;
            }
            m_pimpl->stateConstraintMatrix = constraintMatrix;
            return true;
        }

        bool LinearConstraint::setControlConstraintMatrix(std::shared_ptr<TimeVaryingMatrix> constraintMatrix)
        {
            if (!constraintMatrix) {
                reportError(name().c_str(), "setControlConstraintMatrix", "Empty constraintMatrix pointer.");
                return false;
            }
            m_pimpl->controlConstraintMatrix = constraintMatrix;
            return true;
        }


        bool LinearConstraint::evaluateConstraint(double time,
                                                  const VectorDynSize& state,
                                                  const VectorDynSize& control,
                                                  VectorDynSize& constraint)
        {
            if (m_pimpl->constrainsState) {
                bool isValid = false;
                const MatrixDynSize& stateConstraintMatrix = m_pimpl->stateConstraintMatrix->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state constraint matrix at time: " << time << ".";
                    reportError(name().c_str(), "evaluateConstraint", errorMsg.str().c_str());
                    return false;
                }

                if ((stateConstraintMatrix.cols() != state.size()) || (stateConstraintMatrix.rows() != constraintSize())) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state constraint matrix at time: " << time << " has dimensions not matching with the specified state space dimension or constraint dimension.";
                    reportError(name().c_str(), "evaluateConstraint", errorMsg.str().c_str());
                }
                iDynTree::iDynTreeEigenConstMatrixMap stateConstraint = iDynTree::toEigen(stateConstraintMatrix);

                iDynTree::toEigen(m_pimpl->stateConstraintsBuffer) = stateConstraint * iDynTree::toEigen(state);
            }


            if (m_pimpl->constrainsControl) {
                bool isValid = false;
                const MatrixDynSize& controlConstraintMatrix = m_pimpl->controlConstraintMatrix->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control constraint matrix at time: " << time << ".";
                    reportError(name().c_str(), "evaluateConstraint", errorMsg.str().c_str());
                    return false;
                }

                if ((controlConstraintMatrix.cols() != control.size()) || (controlConstraintMatrix.rows() != constraintSize())) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control constraint matrix at time: " << time << " has dimensions not matching with the specified control space dimension or constraint dimension.";
                    reportError(name().c_str(), "evaluateConstraint", errorMsg.str().c_str());
                }

                iDynTree::iDynTreeEigenConstMatrixMap controlConstraint = iDynTree::toEigen(controlConstraintMatrix);
                iDynTree::toEigen(m_pimpl->controlConstraintsBuffer) = controlConstraint * iDynTree::toEigen(control);
            }

            constraint.resize(static_cast<unsigned int>(constraintSize()));
            toEigen(constraint) = iDynTree::toEigen(m_pimpl->stateConstraintsBuffer) + iDynTree::toEigen(m_pimpl->controlConstraintsBuffer); //the buffers are zero if not constrained

            return true;

        }

        bool LinearConstraint::constraintJacobianWRTState(double time,
                                                const VectorDynSize& state,
                                                const VectorDynSize& /*control*/,
                                                MatrixDynSize& jacobian)
        {
            jacobian.resize(static_cast<unsigned int>(constraintSize()), state.size());

            if (m_pimpl->constrainsState) {
                bool isValid = false;
                const MatrixDynSize& stateConstraintMatrix = m_pimpl->stateConstraintMatrix->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid state constraint matrix at time: " << time << ".";
                    reportError(name().c_str(), "constraintJacobianWRTState", errorMsg.str().c_str());
                    return false;
                }

                if ((stateConstraintMatrix.cols() != state.size()) || (stateConstraintMatrix.rows() != constraintSize())) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state constraint matrix at time: " << time << " has dimensions not matching with the specified state space dimension or constraint dimension.";
                    reportError(name().c_str(), "constraintJacobianWRTState", errorMsg.str().c_str());
                }

                jacobian = stateConstraintMatrix;
            } else {
                jacobian.zero();
            }

            return true;
        }

        bool LinearConstraint::constraintJacobianWRTControl(double time,
                                                  const VectorDynSize& /*state*/,
                                                  const VectorDynSize& control,
                                                  MatrixDynSize& jacobian)
        {
            jacobian.resize(static_cast<unsigned int>(constraintSize()), control.size());

            if (m_pimpl->constrainsControl) {
                bool isValid = false;
                const MatrixDynSize& controlConstraintMatrix = m_pimpl->controlConstraintMatrix->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid control constraint matrix at time: " << time << ".";
                    reportError(name().c_str(), "constraintJacobianWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if ((controlConstraintMatrix.cols() != control.size()) || (controlConstraintMatrix.rows() != constraintSize())) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control constraint matrix at time: " << time << " has dimensions not matching with the specified control space dimension or constraint dimension.";
                    reportError(name().c_str(), "constraintJacobianWRTControl", errorMsg.str().c_str());
                }

                jacobian = controlConstraintMatrix;
            } else {
                jacobian.zero();
            }

            return true;
        }

        bool LinearConstraint::constraintJacobianWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            if (!m_pimpl->hasStateSparsity) {
                return false;
            }
            stateSparsity = m_pimpl->stateSparsity;
            return true;
        }

        bool LinearConstraint::constraintJacobianWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            if (!m_pimpl->hasControlSparsity) {
                return false;
            }
            controlSparsity = m_pimpl->controlSparsity;
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTState(double /*time*/, const VectorDynSize &state, const VectorDynSize &/*control*/, const VectorDynSize &/*lambda*/, MatrixDynSize &hessian)
        {
            hessian.resize(state.size(), state.size());
            hessian.zero();
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &control, const VectorDynSize &/*lambda*/, MatrixDynSize &hessian)
        {
            hessian.resize(control.size(), control.size());
            hessian.zero();
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTStateControl(double /*time*/, const VectorDynSize &state, const VectorDynSize &control, const VectorDynSize &/*lambda*/, MatrixDynSize &hessian)
        {
            hessian.resize(state.size(), control.size());
            hessian.zero();
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            stateSparsity.clear();
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &stateControlSparsity)
        {
            stateControlSparsity.clear();
            return true;
        }

        bool LinearConstraint::constraintSecondPartialDerivativeWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            controlSparsity.clear();
            return true;
        }

    }
}
