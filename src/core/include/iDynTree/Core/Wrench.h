/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_WRENCH_H
#define IDYNTREE_WRENCH_H

#include <iDynTree/Core/SpatialForceVector.h>

namespace iDynTree
{
    /**
     * Class representing a wrench, i.e. a 6D combination of linear force an angular torque.
     *
     * \ingroup iDynTreeCore
     *
     */
    class Wrench: public SpatialForceVector
    {
    public:
        inline Wrench() {}
        Wrench(const Force & _linearVec3, const Torque & _angularVec3);
        Wrench(const SpatialForceVector & other);
        Wrench(const Wrench & other);

        // overloaded operators
        Wrench operator+(const Wrench &other) const;
        Wrench operator-(const Wrench &other) const;
        Wrench operator-() const;
    };
}

#endif
