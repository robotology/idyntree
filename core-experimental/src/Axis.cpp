/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <sstream>

namespace iDynTree
{

    Axis::Axis()
    {
        this->setToDefault();
    }

    Axis::Axis(const Direction& _direction, const Position& _origin):
               direction(_direction), origin(_origin)
    {

    }

    Axis::Axis(const Axis& other):
              direction(other.getDirection()), origin(other.getOrigin())
    {

    }

    Axis::~Axis()
    {

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
        direction = Direction();
        origin    = Position();
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