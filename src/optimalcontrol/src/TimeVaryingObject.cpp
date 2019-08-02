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
