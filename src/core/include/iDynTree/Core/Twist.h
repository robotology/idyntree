/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
