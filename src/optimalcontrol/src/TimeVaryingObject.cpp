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

#include <iDynTree/TimeVaryingObject.h>

namespace iDynTree {
    namespace optimalcontrol {

        TimeInvariantVector::TimeInvariantVector()
        { }

        TimeInvariantVector::TimeInvariantVector(const VectorDynSize &timeInvariantVector)
            : m_timeInvariantVector(timeInvariantVector)
        { }

        VectorDynSize &TimeInvariantVector::get()
        {
            return m_timeInvariantVector;
        }

        const VectorDynSize &TimeInvariantVector::getObject(double /*time*/, bool &isValid)
        {
            isValid = true;
            return m_timeInvariantVector;
        }

        TimeInvariantMatrix::TimeInvariantMatrix()
        { }

        TimeInvariantMatrix::TimeInvariantMatrix(const MatrixDynSize &timeInvariantMatrix)
            : m_timeInvariantMatrix(timeInvariantMatrix)
        { }

        MatrixDynSize &TimeInvariantMatrix::get()
        {
            return m_timeInvariantMatrix;
        }

        const MatrixDynSize &TimeInvariantMatrix::getObject(double /*time*/, bool &isValid)
        {
            isValid = true;
            return m_timeInvariantMatrix;
        }
    }
}
