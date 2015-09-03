/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
     * Currently this class does not support semantics.
     */
    class Wrench: public SpatialForceVector
    {
    public:
        Wrench();
        Wrench(const Force & _linearVec3, const Torque & _angularVec3);
        Wrench(const SpatialForceVector & other);
        Wrench(const Wrench & other);
        virtual ~Wrench();

        // overloaded operators
        Wrench operator+(const Wrench &other) const;
        Wrench operator-(const Wrench &other) const;
        Wrench operator-() const;
    };
}

#endif
