/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TWIST_H
#define IDYNTREE_TWIST_H


#include  <iDynTree/Core/SpatialMotionVector.h>

namespace iDynTree
{
    class SpatialAcc;
    class SpatialMomentum;
    class Wrench;

    /**
     * Class representing a twist, i.e. a 6D combination of linear an angular velocity.
     *
     * Currently this class does not support semantics.
     *
     * \ingroup iDynTreeCore
     */
    class Twist: public SpatialMotionVector
    {
    public:
        Twist();
        Twist(const LinVelocity & _linearVec3, const AngVelocity & _angularVec3);
        Twist(const SpatialMotionVector& other);
        Twist(const Twist& other);
        virtual ~Twist();

        /** overloaded operators **/
        Twist operator+(const Twist &other) const;
        Twist operator-(const Twist &other) const;
        Twist operator-() const;

        /** overloaded cross products */
        SpatialAcc operator*(const Twist &other) const;
        Wrench operator*(const SpatialMomentum &other) const;

    };
}

#endif
