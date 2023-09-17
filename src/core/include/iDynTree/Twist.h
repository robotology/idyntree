// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_TWIST_H
#define IDYNTREE_TWIST_H


#include  <iDynTree/SpatialMotionVector.h>

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
        Twist& operator=(const Twist & other);
        Twist operator+(const Twist &other) const;
        Twist operator-(const Twist &other) const;
        Twist operator-() const;

        /** overloaded cross products */
        SpatialAcc operator*(const Twist &other) const;
        Wrench operator*(const SpatialMomentum &other) const;

    };
}

#endif
