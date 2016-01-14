/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>


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
        return SpatialMotionForceVectorT(other.getLinearVec3().changePoint(pos, other.getAngularVec3()),
                                         other.getAngularVec3());
    }

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changePointOfForceT(const Position & pos,
                                                         const SpatialMotionForceVectorT & other)
    {
        return SpatialMotionForceVectorT(other.getLinearVec3(),
                                         other.getAngularVec3().changePoint(pos, other.getLinearVec3()));
    }

    /**
     * class Method definitions
     */

    // For all the constructors and functions below, checking the semantics while debugging
    // should always be done before the actual composition.

    Position::Position(): PositionRaw()
    {
    }

    Position::Position(double x, double y, double z): PositionRaw(x,y,z)
    {
    }

    Position::Position(const Position & other): PositionRaw(other)
    {
        iDynTreeSemanticsOp(this->semantics = other.getSemantics());
    }

    Position::Position(const PositionRaw& other): PositionRaw(other)
    {

    }

    Position::Position(const PositionRaw & otherPos, const PositionSemantics & otherSem): PositionRaw(otherPos)
    {
        iDynTreeSemanticsOp(this->semantics = otherSem);
    }


    PositionSemantics& Position::getSemantics()
    {
        return this->semantics;
    }

    const PositionSemantics& Position::getSemantics() const
    {
        return this->semantics;
    }


    const Position& Position::changePoint(const Position& newPoint)
    {
        iDynTreeAssert( this->semantics.changePoint(newPoint.semantics) );
        this->PositionRaw::changePoint(newPoint);
        return *this;
    }

    const Position& Position::changeRefPoint(const Position& newRefPoint)
    {
        iDynTreeAssert( this->semantics.changeRefPoint(newRefPoint.semantics) );
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
        PositionSemantics resultSemantics;
        iDynTreeAssert( PositionSemantics::compose(op1.semantics,op2.semantics,resultSemantics) );
        return Position(PositionRaw::compose(op1,op2),resultSemantics);
    }


    Position Position::inverse(const Position& op)
    {
        PositionSemantics resultSemantics;
        iDynTreeAssert( PositionSemantics::inverse(op.semantics,resultSemantics) );
        return Position(PositionRaw::inverse(op),resultSemantics);
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
        iDynTreeSemanticsOp(ss << " " << semantics.toString());

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
