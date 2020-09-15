/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    /**
     * Local static functions
     */

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changePointOfMotionT(const Position & pos,
                                                          const SpatialMotionForceVectorT & other)
    {
        GeomVector3 newLinearVec;
        toEigen(newLinearVec) = toEigen(other.getLinearVec3()) + toEigen(pos).cross(toEigen(other.getAngularVec3()));
        return SpatialMotionForceVectorT(newLinearVec,
                                         other.getAngularVec3());
    }

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changePointOfForceT(const Position & pos,
                                                         const SpatialMotionForceVectorT & other)
    {
        GeomVector3 newAngularVec;
        toEigen(newAngularVec) = toEigen(other.getAngularVec3()) + toEigen(pos).cross(toEigen(other.getLinearVec3()));
        return SpatialMotionForceVectorT(other.getLinearVec3(),
                                         newAngularVec);
    }

    /**
     * class Method definitions
     */

    Position::Position(): PositionRaw()
    {
    }

    Position::Position(double x, double y, double z): PositionRaw(x,y,z)
    {
    }

    Position::Position(const Position & other): PositionRaw(other)
    {
    }

    Position::Position(const PositionRaw& other): PositionRaw(other)
    {

    }

    Position::Position(Span<const double> other): PositionRaw(other)
    {
    }

    const Position& Position::changePoint(const Position& newPoint)
    {
        this->PositionRaw::changePoint(newPoint);
        return *this;
    }

    const Position& Position::changeRefPoint(const Position& newRefPoint)
    {
        this->PositionRaw::changeRefPoint(newRefPoint);
        return *this;
    }

    const Position& Position::changeCoordinateFrame(const Rotation & newCoordinateFrame)
    {
        *this = newCoordinateFrame.changeCoordFrameOf(*this);
        return *this;
    }

    Position Position::compose(const Position& op1, const Position& op2)
    {
        return Position(PositionRaw::compose(op1,op2));
    }


    Position Position::inverse(const Position& op)
    {
        return Position(PositionRaw::inverse(op));
    }

    SpatialMotionVector Position::changePointOf(const SpatialMotionVector & other) const
    {
        return changePointOfMotionT<SpatialMotionVector>(*this, other);
    }

    SpatialForceVector Position::changePointOf(const SpatialForceVector & other) const
    {
        return changePointOfForceT<SpatialForceVector>(*this, other);
    }

    Twist Position::changePointOf(const Twist & other) const
    {
        return changePointOfMotionT<Twist>(*this, other);
    }

    SpatialAcc Position::changePointOf(const SpatialAcc & other) const
    {
        return changePointOfMotionT<SpatialAcc>(*this, other);
    }

    Wrench Position::changePointOf(const Wrench & other) const
    {
        return changePointOfForceT<Wrench>(*this, other);
    }

    SpatialMomentum Position::changePointOf(const SpatialMomentum & other) const
    {
        return changePointOfForceT<SpatialMomentum>(*this, other);
    }

    // overloaded operators
    Position Position::operator+(const Position& other) const
    {
        return compose(*this,other);
    }

    Position Position::operator-(const Position& other) const
    {
        return compose(*this,inverse(other));
    }

    Position Position::operator-() const
    {
        return inverse(*this);
    }

    Twist Position::operator*(const Twist& other) const
    {
        return changePointOfMotionT<Twist>(*this, other);
    }

    SpatialAcc Position::operator*(const SpatialAcc& other) const
    {
        return changePointOfMotionT<SpatialAcc>(*this, other);
    }

    SpatialForceVector Position::operator*(const SpatialForceVector& other) const
    {
        return changePointOfForceT<SpatialForceVector>(*this, other);
    }

    SpatialMomentum Position::operator*(const SpatialMomentum& other) const
    {
        return changePointOfForceT<SpatialMomentum>(*this, other);
    }

    Wrench Position::operator*(const Wrench& other) const
    {
        return changePointOfForceT<Wrench>(*this, other);
    }

    std::string Position::toString() const
    {
        std::stringstream ss;

        ss << PositionRaw::toString();
        return ss.str();
    }

    std::string Position::reservedToString() const
    {
        return this->toString();
    }

    Position Position::Zero()
    {
        Position ret;
        ret.zero();
        return ret;
    }


}
