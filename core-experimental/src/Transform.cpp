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
#include "SpatialMomentum.h"
#include "SpatialAcc.h"
#include "SpatialInertia.h"

#include "PrivateUtils.h"
#include "Utils.h"

#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <sstream>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{

/**
 * Static functions
 * 
 * We define here templated functions implementing geometrical operations, listed below
 * (with the respective instanciation result):
 * 
 * - geometric operations on 3x1 vectors (positions and rotations and homogemeous tranform)
 *
 *   static Position transform(const Transform & op1, const Position & op2)
 *
 * - geometric operations on spatial 6x1 vectors (Twist, SpatialAcc, SpatialMomentum, Wrench).
 *   The distinction between these vectors is handled in Position and Rotation operations
 *   through overloaded functions.
 *
 *   static Wrench   transform(const Transform & op1, const Wrench   & op2)
 *   static Twist    transform(const Transform & op1, const Twist    & op2)
 *   static SpatialMomentum transform(const Transform & op1, const SpatialMomentum & op2)
 *   static SpatialAcc      transform(const Transform & op1, const SpatialAcc & op2)
 *   
 * - geometric operations on spatial 6x6 matrices (SpatialInertia).
 *
 *   static SpatialInertia  transform(const Transform & op1, const SpatialInertia & op2)
 */

template<class T>
static T transform(const Transform & op1, const T & op2);

template<>
Position transform(const Transform& op1, const Position& op2);

template<>
SpatialInertia transform(const Transform& op1, const SpatialInertia& op2);

    
/**
 * Class functions
 */

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
    // and set the semantics. Here we could just have done the check,
    // without setting the semantics, because they are set in the
    // processing that follows.
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
    return transform<Position>(*this,op2);
}

Twist Transform::operator*(const Twist& op2) const
{
    return transform<Twist>(*this,op2);
}

Wrench Transform::operator*(const Wrench& op2) const
{
    return transform<Wrench>(*this,op2);
}

SpatialMomentum Transform::operator*(const SpatialMomentum& op2) const
{
    return transform<SpatialMomentum>(*this,op2);
}

SpatialAcc Transform::operator*(const SpatialAcc& op2) const
{
    return transform<SpatialAcc>(*this,op2);
}


SpatialInertia Transform::operator*(const SpatialInertia& op2) const
{
    return transform<SpatialInertia>(*this,op2);
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


/**
 * Static functions
 * 
 *
 */
    template<class T>
    T transform(const Transform& op1, const T& op2)
    {
        /*
         *### Twist Transform::transform(const Transform& op1, const Twist& op2)
         *
         * The Twist is a Spatial Motion vector.
         * We decompose the spatial transform as the product of the translation
         * and the rotation:
         * B^X_A = |R   0|*|1   0|
         *         |0   R| |p˜' 1|
         *
         * A^X_B = |1   0|*|R'  0 | = xlt(-p) * rot(R')
         *         |p˜  1| |0   R'|
         * where R is the rotation transforming coordinates from [A] to [B],
         * R' is the rotation transforming coordinates from [B] to [A] (op1.rot),
         * p the position of B with respect to A (\vec{AB} expressed in Frame A coordinates, op1.pos)
         * xlt(-p) the spatial translation component of A^X_B = xlt(op1.pos)
         * rot(R') the spatial rotation component of A^X_B = rot(op1.rot)
         * 
         * (For more details, refer to Featherstone's Rigid Body Dynamics Algorithms, 2.8 Coordinate Transforms)
         * 
         * so, considering a given "twist":
         * A^X_B * twist = xlt(op1.pos) * rot(op1.rot) * twist
         *
         * If we associate (xlt(op1.pos)*) to the operator Position::operator * (Twist&),
         * and (rot(op1.rot)*) to the operator Rotation::operator * (Twist&), we then get:
         *
         * op1 * twist = op1.pos * (op1.rot * op2)
         *
         *### Wrench Transform::transform(const Transform& op1, const Wrench& op2)
         *
         * Same thing as for Twist, but with translation and rotation for a Spatial Force vector:
         *
         * A^X_B^* = xlt(p)' * rot(R')
         *
         * Position::operator * (Twist&) |-> (xlt(p)' *)
         * Rotation::operator * (Twist&) |-> (rot(R') *)
         *
         *### SpatialMomentum Transform::transform(const Transform& op1, const SpatialMomentum& op2)
         *
         * The Spatial Momentum is a Spatial Force vector
         *
         *###SpatialAcc Transform::transform(const Transform& op1, const SpatialAcc& op2)
         *
         * The Spatial Acceleration is a Spatial Motion vector
         *
         */
        
        return op1.getPosition()*(op1.getRotation()*op2);
    }
    
    template<>
    Position transform(const Transform& op1, const Position& op2)
    {
        /* op2 is a vector of dimention 3. We expand here the dot product
         * between the homogeneous transform and the homogeneous position
         * (vector of dim 4), as being : [T].[x y z 1]'
         */
        return (op1.getRotation()*op2+op1.getPosition());
    }

    template<>
    SpatialInertia transform(const Transform& op1, const SpatialInertia& op2)
    {
        /* The transform of a Spatial Inertia is defined as follows:
         * A^I = A^X_B^* * B^I * B^X_A
         */
        
        // mass clearly remains the same
        double newMass = op2.getMass();
        
        // the com is transformed as any position
        Position newCenterOfMass = transform<Position>(op1,Position(op2.getCenterOfMass()));
        
        // the rotational inertial is rotated and then
        // the parallel axis theorem applies
        RotationalInertiaRaw newRotInertia;
        Eigen::Map<Eigen::Matrix3d> newI(newRotInertia.data());
        RotationalInertiaRaw oldRotInertia = op2.getRotationalInertiaWrtFrameOrigin();
        Eigen::Map<const Eigen::Matrix3d> oldI(oldRotInertia.data());
        
        Eigen::Map<const Matrix3dRowMajor> R(op1.getRotation().data());
        Eigen::Map<const Eigen::Vector3d> p(op1.getPosition().data());
        
        newI =  R*oldI*R.transpose() - newMass*squareCrossProductMatrix(p);
        
        return SpatialInertia(newMass,newCenterOfMass,newRotInertia);
    }

}
