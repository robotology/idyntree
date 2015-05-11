/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Transform.h"
#include "Position.h"
#include "Rotation.h"
#include "Twist.h"
#include "Wrench.h"

#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

Transform::Transform(): TransformRaw()
{
}

Transform::Transform(const Transform& other): TransformRaw(other),
                                              semantics(other.getSemantics())
{
}

Transform::Transform(const TransformRaw& other): TransformRaw(other)
{

}

Transform::Transform(const Rotation& rot, const Position& origin)
{
    // warning: this could cause errors
    this->setRotation(rot);
    this->setPosition(origin);
}

Transform::~Transform()
{

}

const Position Transform::getPosition() const
{
    Position result = this->pos;
    result.getSemantics().setCoordinateFrame(this->semantics.getReferenceOrientationFrame());
    result.getSemantics().setPoint(this->semantics.getPoint());
    result.getSemantics().setReferencePoint(this->semantics.getReferencePoint());
    return result;
}

const Rotation Transform::getRotation() const
{
    Rotation result = this->rot;
    result.getSemantics().setOrientationFrame(this->semantics.getOrientationFrame());
    result.getSemantics().setReferenceOrientationFrame(this->semantics.getReferenceOrientationFrame());
    return result;
}

const bool Transform::setPosition(const Position& position)
{
    this->pos = position;
    return this->semantics.setPositionSemantics(position.getSemantics());
}

const bool Transform::setRotation(const Rotation& rotation)
{
    this->rot = rotation;
    return this->semantics.setRotationSemantics(rotation.getSemantics());
}

const bool Transform::setPosition(const PositionRaw& position)
{
    setPosition(iDynTree::Position(position));
}

const bool Transform::setRotation(const RotationRaw& rotation)
{
    setRotation(iDynTree::Rotation(rotation));
}


TransformSemantics& Transform::getSemantics()
{
    return this->semantics;
}

const TransformSemantics& Transform::getSemantics() const
{
    return this->semantics;
}

Transform Transform::compose(const Transform& op1, const Transform& op2)
{
    Transform result;

    if( TransformSemantics::check_compose(op1.getSemantics(),op2.getSemantics()) )
    {
        result = TransformRaw::compose(op1,op2);
        TransformSemantics::compose(op1.getSemantics(),op2.getSemantics(),result.getSemantics());
    }

    return result;
}

Transform Transform::inverse2(const Transform& trans)
{
    Transform result;

    if( TransformSemantics::check_inverse2(trans.getSemantics()) )
    {
        result = TransformRaw::inverse2(trans);
        TransformSemantics::inverse2(trans.getSemantics(),result.getSemantics());
    }

    return result;
}

Position Transform::transform(const Transform& op1, const Position& op2)
{
    Position result;

    if( TransformSemantics::check_transform(op1.getSemantics(),op2.getSemantics()) )
    {
        result = TransformRaw::transform(op1,op2);
        TransformSemantics::transform(op1.getSemantics(),op2.getSemantics(),result.getSemantics());
    }

    return result;
}

Twist Transform::transform(const Transform& op1, const Twist& op2)
{
    Twist result;

    // \todo TODO add semantics to Twist
    result = TransformRaw::transform(op1,op2);

    return result;
}

Wrench Transform::transform(const Transform& op1, const Wrench& op2)
{
    Wrench result;

    // \todo TODO add semantics to Twist
    result = TransformRaw::transform(op1,op2);

    return result;
}



Transform Transform::operator*(const Transform& other) const
{
    return compose(*this,other);
}

Transform Transform::inverse() const
{
    return Transform::inverse2(*this);
}


Position Transform::operator*(const Position& op2) const
{
    return Transform::transform(*this,op2);
}

Twist Transform::operator*(const Twist& op2) const
{
    return Transform::transform(*this,op2);
}

Wrench Transform::operator*(const Wrench& op2) const
{
    return Transform::transform(*this,op2);
}

std::string Transform::toString() const
{
    std::stringstream ss;

    ss << TransformRaw::toString() << " " << semantics.toString();

    return ss.str();
}

std::string Transform::reservedToString() const
{
    return this->toString();
}



}
