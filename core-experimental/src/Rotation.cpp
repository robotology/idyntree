/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Rotation.h"
#include "Position.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

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
        if( semantics.check_changeOrientFrame(newOrientFrame.getSemantics()) )
        {
            RotationRaw::changeOrientFrame(newOrientFrame);
            semantics.changeOrientFrame(newOrientFrame.getSemantics());
        }

        return *this;
    }

    const Rotation& Rotation::changeRefOrientFrame(const Rotation& newRefOrientFrame)
    {
        if( semantics.check_changeRefOrientFrame(newRefOrientFrame.getSemantics()) )
        {
            RotationRaw::changeRefOrientFrame(newRefOrientFrame);
            semantics.changeRefOrientFrame(newRefOrientFrame.getSemantics());
        }

        return *this;
    }

    Rotation Rotation::compose(const Rotation& op1, const Rotation& op2)
    {
        Rotation result;

        if( RotationSemantics::check_compose(op1.getSemantics(),op2.getSemantics()) )
        {
            result = RotationRaw::compose(op1,op2);
            RotationSemantics::compose(op1.getSemantics(),op2.getSemantics(),result.getSemantics());
        }

        return result;
    }

    Rotation Rotation::inverse2(const Rotation& orient)
    {
        Rotation result;

        if( RotationSemantics::check_inverse2(orient.getSemantics()) )
        {
            result = RotationRaw::inverse2(orient);
            RotationSemantics::inverse2(orient.getSemantics(),result.getSemantics());
        }

        return result;
    }

    Position Rotation::transform(const Rotation& op1, const Position& op2)
    {
        Position result;

        if( RotationSemantics::check_transform(op1.getSemantics(),op2.getSemantics()) )
        {
            result = RotationRaw::transform(op1,op2);
            RotationSemantics::transform(op1.getSemantics(),op2.getSemantics(),result.getSemantics());
        }

        return result;
    }


    Rotation Rotation::inverse() const
    {
        return Rotation::inverse2(*this);
    }

    Rotation Rotation::operator*(const Rotation& other) const
    {
        return compose(*this,other);
    }

    Position Rotation::operator*(const Position& op) const
    {
        return transform(*this,op);
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
