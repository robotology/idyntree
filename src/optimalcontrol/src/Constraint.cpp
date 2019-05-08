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

#include <iDynTree/Constraint.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/MatrixDynSize.h>

#include <cassert>

namespace iDynTree {
    namespace optimalcontrol {

        Constraint::Constraint(size_t size, const std::string name)
        : m_constraintSize(size)
        , m_constraintName(name)
        , m_valueBuffer(static_cast<unsigned int>(size))
        , m_isLowerBounded(false)
        , m_isUpperBounded(false)
        {
            m_valueBuffer.zero();
            m_lowerBound.resize(static_cast<unsigned int>(size));
            m_upperBound.resize(static_cast<unsigned int>(size));
        }

        Constraint::~Constraint() {}

        size_t Constraint::constraintSize() const { return m_constraintSize; }

        const std::string &Constraint::name() const { return m_constraintName; }

        bool Constraint::setLowerBound(const VectorDynSize &lowerBound)
        {
            if (lowerBound.size() != m_constraintSize){
                reportError("Constraint", "setLowerBound", "The lowerBound dimension is not coherent with the constraint size.");
                return false;
            }
            m_lowerBound = lowerBound;
            m_isLowerBounded = true;
            return true;
        }

        bool Constraint::getLowerBound(VectorDynSize &lowerBound)
        {
            if (!m_isLowerBounded) {
                    return false;
            }
            if (lowerBound.size() != m_lowerBound.size()) {
                lowerBound.resize(m_lowerBound.size());
            }
            lowerBound = m_lowerBound;
            return true;
        }

        bool Constraint::setUpperBound(const VectorDynSize &upperBound)
        {
            if (upperBound.size() != m_constraintSize) {
                reportError("Constraint", "setUpperBound", "The upperBound dimension is not coherent with the constraint size.");
                return false;
            }
            m_upperBound = upperBound;
            m_isUpperBounded = true;
            return true;
        }

        bool Constraint::getUpperBound(VectorDynSize &upperBound)
        {
            if (!m_isUpperBounded) {
                    return false;
            }
            if (upperBound.size() != m_upperBound.size()) {
                upperBound.resize(m_upperBound.size());
            }
            upperBound = m_upperBound;
            return true;
        }

        bool Constraint::isFeasiblePoint(double time, const VectorDynSize &state, const VectorDynSize &control)
        {
            if (!m_isUpperBounded && !m_isLowerBounded) {
                return true;
            }

            if (!evaluateConstraint(time, state, control, m_valueBuffer)){
                reportError("Constraint", "isFeasiblePoint", "Failed to evaluate constraint.");
                assert(false);
            }

            for (unsigned int i = 0; i < constraintSize(); ++i){
                if (m_isLowerBounded) {
                    if (m_valueBuffer(i) < m_lowerBound(i)) {
                        return false;
                    }
                }

                if (m_isUpperBounded) {
                    if (m_valueBuffer(i) > m_upperBound(i)) {
                        return false;
                    }
                }
            }
            return true;
        }

        bool Constraint::constraintJacobianWRTState(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*control*/, MatrixDynSize &/*jacobian*/)
        {
            return false;
        }

        bool Constraint::constraintJacobianWRTControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*control*/, MatrixDynSize &/*jacobian*/)
        {
            return false;
        }

        size_t Constraint::expectedStateSpaceSize() const
        {
            return 0;
        }

        size_t Constraint::expectedControlSpaceSize() const
        {
            return 0;
        }

        bool Constraint::constraintJacobianWRTStateSparsity(SparsityStructure &/*stateSparsity*/)
        {
            return false;
        }

        bool Constraint::constraintJacobianWRTControlSparsity(SparsityStructure &/*controlSparsity*/)
        {
            return false;
        }

        bool Constraint::constraintSecondPartialDerivativeWRTState(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*control*/, const VectorDynSize &/*lambda*/, MatrixDynSize &/*hessian*/)
        {
            return false;
        }


        bool Constraint::constraintSecondPartialDerivativeWRTControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*control*/, const VectorDynSize &/*lambda*/, MatrixDynSize &/*hessian*/)
        {
            return false;
        }

        bool Constraint::constraintSecondPartialDerivativeWRTStateControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*control*/, const VectorDynSize &/*lambda*/, MatrixDynSize &/*hessian*/)
        {
            return false;
        }

        bool Constraint::constraintSecondPartialDerivativeWRTStateSparsity(SparsityStructure &/*stateSparsity*/)
        {
            return false;
        }

        bool Constraint::constraintSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &/*stateControlSparsity*/)
        {
            return false;
        }

        bool Constraint::constraintSecondPartialDerivativeWRTControlSparsity(SparsityStructure &/*controlSparsity*/)
        {
            return false;
        }

    }
}
