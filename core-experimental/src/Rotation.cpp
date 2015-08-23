/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Rotation.h"
#include "Position.h"
#include "Wrench.h"
#include "Twist.h"
#include "SpatialAcc.h"
#include "SpatialMomentum.h"
#include "Utils.h"



#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    /**
     * Local static functions
     */
    
    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changeCoordFrameOfT(const Rotation & rot,
                                                         const SpatialMotionForceVectorT & other)
    {
        return SpatialMotionForceVectorT(other.getLinearVec3().changeCoordFrame(rot),
                                         other.getAngularVec3().changeCoordFrame(rot));
    }

    /**
     * class Method definitions
     */
    
    Rotation::Rotation(): RotationRaw()
    {
    }

    Rotation::Rotation(double xx, double xy, double xz,
                       double yx, double yy, double yz,
                       double zx, double zy, double zz): RotationRaw(xx,xy,xz,
                                                                     yx,yy,yz,
                                                                     zx,zy,zz)
    {
    }

    Rotation::Rotation(const Rotation & other): RotationRaw(other)
    {
        this->semantics = other.getSemantics();
    }

    Rotation::Rotation(const RotationRaw& other): RotationRaw(other)
    {

    }

    Rotation::Rotation(const RotationRaw& otherPos, RotationSemantics & otherSem): RotationRaw(otherPos)
    {
        this->semantics = otherSem;
    }

    Rotation::~Rotation()
    {
    }

    RotationSemantics& Rotation::getSemantics()
    {
        return this->semantics;
    }

    const RotationSemantics& Rotation::getSemantics() const
    {
        return this->semantics;
    }

    const Rotation& Rotation::changeOrientFrame(const Rotation& newOrientFrame)
    {
        iDynTreeAssert( this->semantics.changeOrientFrame(newOrientFrame.semantics) );
        this->RotationRaw::changeOrientFrame(newOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeRefOrientFrame(const Rotation& newRefOrientFrame)
    {
        iDynTreeAssert( this->semantics.changeRefOrientFrame(newRefOrientFrame.semantics) );
        this->RotationRaw::changeRefOrientFrame(newRefOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeCoordinateFrame(const Rotation& newCoordinateFrame)
    {
        return this->changeRefOrientFrame(newCoordinateFrame);
    }
    
    Rotation Rotation::compose(const Rotation& op1, const Rotation& op2)
    {
        RotationSemantics resultSemantics;
        iDynTreeAssert( RotationSemantics::compose(op1.semantics,op2.semantics,resultSemantics) );
        return Rotation(RotationRaw::compose(op1,op2),resultSemantics);
    }

    Rotation Rotation::inverse2(const Rotation& orient)
    {
        RotationSemantics resultSemantics;
        iDynTreeAssert( RotationSemantics::inverse2(orient.getSemantics(),resultSemantics) );
        return Rotation(RotationRaw::inverse2(orient),resultSemantics);
    }

    Position Rotation::changeCoordFrameOf(const Position & other) const
    {
        PositionSemantics resultSemantics;
        iDynTreeAssert( this->semantics.changeCoordFrameOf(other.getSemantics(), resultSemantics) );
        return Position(this->RotationRaw::changeCoordFrameOf(other), resultSemantics);
    }
    
    Twist Rotation::changeCoordFrameOf(const Twist& other) const
    {
        return changeCoordFrameOfT<Twist>(*this, other);
    }

    SpatialAcc Rotation::changeCoordFrameOf(const SpatialAcc & other) const
    {
        return changeCoordFrameOfT<SpatialAcc>(*this, other);
    }

    SpatialMomentum Rotation::changeCoordFrameOf(const SpatialMomentum & other) const
    {
        return changeCoordFrameOfT<SpatialMomentum>(*this, other);
    }

    Wrench Rotation::changeCoordFrameOf(const Wrench &other) const
    {
        return changeCoordFrameOfT<Wrench>(*this, other);
    }

    Rotation Rotation::inverse() const
    {
        return inverse2(*this);
    }

    Rotation Rotation::operator*(const Rotation& other) const
    {
        return compose(*this,other);
    }

    Position Rotation::operator*(const Position& other) const
    {
        return changeCoordFrameOf(other);
    }

    Twist Rotation::operator*(const Twist& other) const
    {
        return changeCoordFrameOfT<Twist>(*this, other);
    }

    SpatialAcc Rotation::operator*(const SpatialAcc & other) const
    {
        return changeCoordFrameOfT<SpatialAcc>(*this, other);
    }

    SpatialMomentum Rotation::operator*(const SpatialMomentum & other) const
    {
        return changeCoordFrameOfT<SpatialMomentum>(*this, other);
    }

    Wrench Rotation::operator*(const Wrench& other) const
    {
        return changeCoordFrameOfT<Wrench>(*this, other);
    }

    Rotation Rotation::RotX(const double angle)
    {
        return Rotation(RotationRaw::RotX(angle));
    }
    
    Rotation Rotation::RotY(const double angle)
    {
        return Rotation(RotationRaw::RotY(angle));
    }
    
    Rotation Rotation::RotZ(const double angle)
    {
        return Rotation(RotationRaw::RotZ(angle));
    }
    
    Rotation Rotation::RPY(const double roll, const double pitch, const double yaw)
    {
        return Rotation(RotationRaw::RPY(roll, pitch, yaw));
    }
    
    Rotation Rotation::Identity()
    {
        return Rotation();
    }
    
    std::string Rotation::toString() const
    {
        std::stringstream ss;

        ss << RotationRaw::toString() << " " << semantics.toString();

        return ss.str();
    }

    std::string Rotation::reservedToString() const
    {
        return this->toString();
    }


}
