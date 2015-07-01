/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TWIST_H
#define IDYNTREE_TWIST_H

#include "SpatialMotionVectorRaw.h"

namespace iDynTree
{
    /**
     * Class representing a twist, i.e. a 6D combination of linear an angular velocity.
     *
     * Currently this class does not support semantics.
     *
     * \ingroup iDynTreeCore
     */
    class Twist: public SpatialMotionVectorRaw
    {
    public:
        Twist();
        Twist(const double* in_data, const unsigned int in_size);
        Twist(const SpatialMotionVectorRaw& other);
        Twist(const Twist& other);
        virtual ~Twist();

        /** overloaded operators **/
        Twist operator+(const Twist &other) const;
        Twist operator-(const Twist &other) const;
        Twist operator-() const;
    };
}

#endif