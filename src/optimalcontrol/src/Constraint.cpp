/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/Constraint.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {
    namespace optimalcontrol {

        Constraint::Constraint(size_t size, const std::string name)
        : m_contraintSize(size)
        , m_constraintName(name)
        , m_isLowerBounded(false)
        , m_isUpperBounded(false)
        {
            m_lowerBound.resize(static_cast<unsigned int>(size));
            m_upperBound.resize(static_cast<unsigned int>(size));
        }

        Constraint::~Constraint() {}

        size_t Constraint::constraintSize() const { return m_contraintSize; }

        const std::string &Constraint::name() const { return m_constraintName; }

        bool Constraint::setLowerBound(const VectorDynSize &lowerBound)
        {
            if (lowerBound.size() != m_contraintSize){
                reportError("Constraint", "setLowerBound", "The lowerBound dimension is not coherent with the constraint size.");
                return false;
            }
            m_lowerBound = lowerBound;
            m_isLowerBounded = true;
            return true;
        }

        bool Constraint::getLowerBound(VectorDynSize &lowerBound)
        {
            if (!m_isLowerBounded)
                    return false;
            if (lowerBound.size() != m_lowerBound.size())
                lowerBound.resize(m_lowerBound.size());
            lowerBound = m_lowerBound;
            return true;
        }

        bool Constraint::setUpperBound(const VectorDynSize &upperBound)
        {
            if (upperBound.size() != m_contraintSize){
                reportError("Constraint", "setUpperBound", "The upperBound dimension is not coherent with the constraint size.");
                return false;
            }
            m_upperBound = upperBound;
            m_isUpperBounded = true;
            return true;
        }

        bool Constraint::getUpperBound(VectorDynSize &upperBound)
        {
            if (!m_isUpperBounded)
                    return false;
            if (upperBound.size() != m_upperBound.size())
                upperBound.resize(m_upperBound.size());
            upperBound = m_upperBound;
            return true;
        }

        bool Constraint::constraintJacobianWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &jacobian)
        {
            return false;
        }

        bool Constraint::constraintJacobianWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &jacobian)
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

    }
}
