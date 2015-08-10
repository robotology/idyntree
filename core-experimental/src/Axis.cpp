/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Axis.h>


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

    std::string Axis::toString() const
    {
        std::stringstream ss;

        ss << "Direction: " << direction.toString()
           << " Origin: "   << origin.toString() << std::endl;
    }

    std::string Axis::reservedToString() const
    {
        return this->toString();
    }

}