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

Transform::Transform(): pos(),
                        rot(),
                        semantics(pos.getSemantics(), rot.getSemantics())
{
}

Transform::Transform(const Rotation& _rot, const Position& origin): pos(origin),
                                                                    rot(_rot),
                                                                    semantics(pos.getSemantics(), rot.getSemantics())
{
}

Transform::Transform(const Transform& other): pos(other.getPosition()),
                                              rot(other.getRotation()),
                                              semantics(pos.getSemantics(), rot.getSemantics())
{
}
    
Transform::~Transform()
{

}

TransformSemantics& Transform::getSemantics()
{
    return this->semantics;
}

const TransformSemantics& Transform::getSemantics() const
{
    return this->semantics;
}
    
const Position& Transform::getPosition() const
{
    return this->pos;
}

const Rotation& Transform::getRotation() const
{
    return this->rot;
}

void Transform::setPosition(const Position& position)
{
    // check consistency of setted position with existing rotation
    // and set the semantics.
    iDynTreeAssert(this->semantics.setPositionSemantics(position.getSemantics()));
    
    // set position
    this->pos = position;
}

void Transform::setRotation(const Rotation& rotation)
{
    // check consistency of setted rotation with existing position
    // and set the semantics.
    iDynTreeAssert(this->semantics.setRotationSemantics(rotation.getSemantics()));
    
    // set rotation
    this->rot = rotation;
}


Transform Transform::compose(const Transform& op1, const Transform& op2)
{
    Transform result;
    
    result.setRotation(op1.getRotation()*op2.getRotation());
    result.setPosition(op1.getRotation()*op2.getPosition()+op1.getPosition());
    
    return result;
}

Transform Transform::inverse2(const Transform& trans)
{
    Transform result;
    
    result.setRotation(trans.getRotation().inverse());
    result.setPosition(-(result.getRotation()*trans.getPosition()));
    
    return result;
}

Position Transform::transform(const Transform& op1, const Position& op2)
{
    // op2 is a vector of dimention 3. We expand here the dot product
    // between the homogeneous transform and the homogeneous position
    // (vector of dim 4), as being : [T].[x y z 1]'
    return (op1.getRotation()*op2+op1.getPosition());
}

Twist Transform::transform(const Transform& op1, const Twist& op2)
{
    // We decompose the spatial transform as the product of the translation
    // and the rotation:
    // B^X_A = |E   0|.|1   0|
    //         |0   E| |r˜' 1|
    //
    // A^X_B = |1   0|.|E'  0 | = xlt(r) * rot(E')
    //         |r˜  1| |0   E'|
    // where E is the rotation transforming coordinates from [A] to [B],
    // E' is the rotation transforming coordinates from [B] to [A] (op1.rot),
    // r the position of B with respect to A => op1.pos
    // xlt(r) the spatial translation component of A^X_B = xlt(op1.pos)
    // rot(E') the spatial rotation component of A^X_B = rot(op1.rot)
    //
    // so, considering a given "twist":
    // A^X_B * twist = xlt(op1.pos) * rot(op1.rot) * twist
    //
    // If we associate (xlt(op1.pos)*) to the operator Position::operator * (Twist&),
    // and (rot(op1.rot)*) to the operator Rotation::operator * (Twist&), we then get:
    //
    // op1 * twist = op1.pos * (op1.rot * op2)
    
    return op1.getPosition()*(op1.getRotation()*op2);
}

Wrench Transform::transform(const Transform& op1, const Wrench& op2)
{
    // Same thing as for Twist, but with translation and rotation for a force:
    //
    // A^X_B = xlt(r)' * rot(E')
    //
    // Position::operator * (Twist&) |-> (xlt(r)' *)
    // Rotation::operator * (Twist&) |-> (rot(E') *)
    
    return op1.getPosition()*(op1.getRotation()*op2);
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

Transform Transform::Identity()
{
    return Transform();
}

std::string Transform::toString() const
{
    std::stringstream ss;
    
    ss << rot.toString() << " "
       << pos.toString() << " "
       << semantics.toString() << std::endl;

    return ss.str();
}

std::string Transform::reservedToString() const
{
    return this->toString();
}



}
