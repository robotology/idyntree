// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Axis.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/Twist.h>
#include <iDynTree/SpatialAcc.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <sstream>

namespace iDynTree
{
    Axis::Axis(const Direction& _direction, const Position& _origin):
               direction(_direction), origin(_origin)
    {

    }

    Axis::Axis(const Axis& other):
              direction(other.getDirection()), origin(other.getOrigin())
    {

    }

    Axis& Axis::operator=(const Axis& other)
    {
        this->direction = other.getDirection();
        this->origin = other.getOrigin();
        return *this;
    }

    const Direction& Axis::getDirection() const
    {
        return direction;
    }

    const Position& Axis::getOrigin() const
    {
        return origin;
    }

    void Axis::setDirection(const Direction& _direction)
    {
        direction = _direction;
    }

    void Axis::setOrigin(const Position& _origin)
    {
        origin = _origin;
    }

    void Axis::setToDefault()
    {
        direction = Direction::Default();
        origin    = Position::Zero();
    }

    Transform Axis::getRotationTransform(const double theta) const
    {
        // Formula for rotation around and arbitrary axis given by
        // http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
        Transform nonRotated_T_rotated;

        // rotation
        nonRotated_T_rotated.setRotation(Rotation::RotAxis(this->getDirection(),theta));

        // translation
        double cost = cos(theta);
        double sint = sin(theta);
        double u   = this->getDirection()(0);
        double u2  = u*u;
        double v   = this->getDirection()(1);
        double v2  = v*v;
        double w   = this->getDirection()(2);
        double w2  = w*w;
        double a    = this->getOrigin()(0);
        double b    = this->getOrigin()(1);
        double c    = this->getOrigin()(2);

        Position translationPosition;
        translationPosition(0) =
            (a*(v2+w2) - u*(b*v+c*w))*(1-cost) + (b*w-c*v)*sint;
        translationPosition(1) =
            (b*(u2+w2) - v*(a*u+c*w))*(1-cost) + (c*u-a*w)*sint;
        translationPosition(2) =
            (c*(u2+v2) - w*(a*u+b*v))*(1-cost) + (a*v-b*u)*sint;

        nonRotated_T_rotated.setPosition(translationPosition);

        return nonRotated_T_rotated;
    }
    
    Transform Axis::getTranslationTransform(const double dist) const
    {
        Transform nonTranslated_T_translated;

        //No rotation
        nonTranslated_T_translated.setRotation(iDynTree::Rotation::Identity());

        // translation
        double u   = this->getDirection()(0);
        double v   = this->getDirection()(1);
        double w   = this->getDirection()(2);
       
        Position translationPosition;
            
        //Translation because of joint variable    
        translationPosition(0) = u*dist;
        translationPosition(1) = v*dist;
        translationPosition(2) = w*dist;

        nonTranslated_T_translated.setPosition(translationPosition);

        return nonTranslated_T_translated;
    }

    TransformDerivative Axis::getRotationTransformDerivative(const double theta) const
    {
        // Formula for rotation around and arbitrary axis given by
        // http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
        // In this function we
        TransformDerivative derivative_nonRotated_T_rotated;

        // rotation
        derivative_nonRotated_T_rotated.setRotationDerivative(Rotation::RotAxisDerivative(this->getDirection(),theta));

        // translation
        double cost = cos(theta);
        double sint = sin(theta);
        double u   = this->getDirection()(0);
        double u2  = u*u;
        double v   = this->getDirection()(1);
        double v2  = v*v;
        double w   = this->getDirection()(2);
        double w2  = w*w;
        double a    = this->getOrigin()(0);
        double b    = this->getOrigin()(1);
        double c    = this->getOrigin()(2);

        Vector3 translationPositionDerivative;
        translationPositionDerivative(0) =
            (a*(v2+w2) - u*(b*v+c*w))*(sint) + (b*w-c*v)*cost;
        translationPositionDerivative(1) =
            (b*(u2+w2) - v*(a*u+c*w))*(sint) + (c*u-a*w)*cost;
        translationPositionDerivative(2) =
            (c*(u2+v2) - w*(a*u+b*v))*(sint) + (a*v-b*u)*cost;

        derivative_nonRotated_T_rotated.setPositionDerivative(translationPositionDerivative);

        return derivative_nonRotated_T_rotated;
    }
    
    TransformDerivative Axis::getTranslationTransformDerivative(const double /*dist*/) const
    {
        TransformDerivative derivative_nonTranslated_T_translated;

        //No rotation
        Matrix3x3 zeroRot;
        zeroRot.zero();
        derivative_nonTranslated_T_translated.setRotationDerivative(zeroRot);

        // translation
        double u   = this->getDirection()(0);
        double v   = this->getDirection()(1);
        double w   = this->getDirection()(2);
        
        Vector3 translationPositionDerivative;
            
        //Translation because of joint variable    
        translationPositionDerivative(0) = u;
        translationPositionDerivative(1) = v;
        translationPositionDerivative(2) = w;

        derivative_nonTranslated_T_translated.setPositionDerivative(translationPositionDerivative);

        return derivative_nonTranslated_T_translated;
    }



    Twist Axis::getRotationTwist(const double dtheta) const
    {
        Twist ret;

        Eigen::Map<Eigen::Vector3d> linVel(ret.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> angVel(ret.getAngularVec3().data());

        Eigen::Map<const Eigen::Vector3d> dir(direction.data());
        Eigen::Map<const Eigen::Vector3d> orig(origin.data());

        linVel = dtheta*(orig.cross(dir));
        angVel = dir*dtheta;

        return ret;
    }
    
    Twist Axis::getTranslationTwist(const double ddist) const
    {
        Twist ret;

        Eigen::Map<Eigen::Vector3d> linVel(ret.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> angVel(ret.getAngularVec3().data());

        Eigen::Map<const Eigen::Vector3d> dir(direction.data());
        Eigen::Map<const Eigen::Vector3d> orig(origin.data());

        linVel = dir*ddist;
        angVel << 0,0,0;

        return ret;
    }

    SpatialAcc Axis::getRotationSpatialAcc(const double d2theta) const
    {
        SpatialAcc ret;

        Eigen::Map<Eigen::Vector3d> lin(ret.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> ang(ret.getAngularVec3().data());

        Eigen::Map<const Eigen::Vector3d> dir(direction.data());
        Eigen::Map<const Eigen::Vector3d> orig(origin.data());

        lin = d2theta*(orig.cross(dir));
        ang = dir*d2theta;

        return ret;
    }
    
    SpatialAcc Axis::getTranslationSpatialAcc(const double d2dist) const
    {
        SpatialAcc ret;

        Eigen::Map<Eigen::Vector3d> lin(ret.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> ang(ret.getAngularVec3().data());

        Eigen::Map<const Eigen::Vector3d> dir(direction.data());
        Eigen::Map<const Eigen::Vector3d> orig(origin.data());

        lin = dir*d2dist;
        ang << 0,0,0;

        return ret;
    }
    
    bool Axis::isParallel(const Axis& otherAxis, const double tolerance) const
    {
        return this->direction.isParallel(otherAxis.direction,tolerance);
    }

    Axis Axis::reverse() const
    {
        return Axis(this->getDirection().reverse(),
                    this->getOrigin());
    }

    double Axis::getDistanceBetweenAxisAndPoint(const iDynTree::Position& point) const
    {
        // Calculate the closest point on the axis to the given point
        iDynTree::Position closestPointOnAxis = getPointOnAxisClosestToGivenPoint(point);

        // Calculate the distance between the closest point on the axis and the given point
        return (iDynTree::toEigen(closestPointOnAxis) - iDynTree::toEigen(point)).norm();
    }

    iDynTree::Position Axis::getPointOnAxisClosestToGivenPoint(const iDynTree::Position& point) const
    {
            Eigen::Vector3d direction = iDynTree::toEigen(this->getDirection()).normalized();

            // Vector from the offset point of the to the given point
            Eigen::Vector3d axisOrigin_p_point = iDynTree::toEigen(point) - iDynTree::toEigen(this->getOrigin());

            // Project the axisOrigin_p_point onto the axis direction
            Eigen::Vector3d projection = direction * axisOrigin_p_point.dot(direction);

            // Calculate the closest point on the axis to the given point
            iDynTree::Position closestPointOnAxis;
            iDynTree::toEigen(closestPointOnAxis) = iDynTree::toEigen(this->getOrigin()) + projection;

            return closestPointOnAxis;
    }

    std::string Axis::toString() const
    {
        std::stringstream ss;

        ss << "Direction: " << direction.toString()
           << " Origin: "   << origin.toString() << std::endl;

        return ss.str();
    }

    std::string Axis::reservedToString() const
    {
        return this->toString();
    }

}
