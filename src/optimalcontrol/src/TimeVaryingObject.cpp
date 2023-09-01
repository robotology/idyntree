// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/TimeVaryingObject.h>

namespace iDynTree {
    namespace optimalcontrol {

        template<typename Object>
        TimeInvariantObject<Object>::TimeInvariantObject()
        { }

        template<typename Object>
        TimeInvariantObject<Object>::TimeInvariantObject(const Object &timeInvariantObject)
            : m_invariantObject(timeInvariantObject)
        { }

        template<typename Object>
        TimeInvariantObject<Object>::~TimeInvariantObject()
        { }

        template<typename Object>
        Object &TimeInvariantObject<Object>::get()
        {
            return m_invariantObject;
        }

        template<typename Object>
        const Object &TimeInvariantObject<Object>::get(double /*time*/, bool &isValid)
        {
            isValid = true;
            return m_invariantObject;
        }

        template class TimeInvariantObject<double>;

        template class TimeInvariantObject<VectorDynSize>;

        template class TimeInvariantObject<MatrixDynSize>;

        template class TimeInvariantObject<Transform>;

        template class TimeInvariantObject<Rotation>;

        template class TimeInvariantObject<Position>;
    }
}
