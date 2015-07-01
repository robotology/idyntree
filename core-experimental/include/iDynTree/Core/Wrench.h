/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_WRENCH_H
#define IDYNTREE_WRENCH_H

#include "SpatialForceVectorRaw.h"

namespace iDynTree
{
    /**
     * Class representing a wrench, i.e. a 6D combination of linear force an angular torque.
     *
     * \ingroup iDynTreeCore
     *
     * Currently this class does not support semantics.
     */
    class Wrench: public SpatialForceVectorRaw
    {
    public:
        Wrench();
        Wrench(const double* in_data, const unsigned int in_size);
        Wrench(const SpatialForceVectorRaw & other);
        Wrench(const Wrench & other);
        virtual ~Wrench();

        // overloaded operators
        Wrench operator+(const Wrench &other) const;
        Wrench operator-(const Wrench &other) const;
        Wrench operator-() const;
    };
}

#endif