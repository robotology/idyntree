// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_WRENCH_H
#define IDYNTREE_WRENCH_H

#include <iDynTree/SpatialForceVector.h>

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
        Wrench& operator=(const Wrench & other);
        Wrench operator+(const Wrench &other) const;
        Wrench operator-(const Wrench &other) const;
        Wrench operator-() const;
    };
}

#endif
