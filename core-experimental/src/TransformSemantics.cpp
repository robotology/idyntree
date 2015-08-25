/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/TransformSemantics.h>
#include <iDynTree/Core/RotationSemantics.h>
#include <iDynTree/Core/PositionSemantics.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Utils.h>

#include <iostream>
#include <sstream>

#include <cassert>

namespace iDynTree
{

    TransformSemantics::TransformSemantics(PositionSemantics & position, RotationSemantics & rotation): positionSemantics(position),
                                                                                                        rotationSemantics(rotation)
    {

    }

    TransformSemantics::~TransformSemantics()
    {

    }

    const PositionSemantics & TransformSemantics::getPositionSemantics() const
    {
        return this->positionSemantics;
    }

    const RotationSemantics & TransformSemantics::getRotationSemantics() const
    {
        return this->rotationSemantics;
    }

    bool TransformSemantics::check_position2rotationConsistency(const PositionSemantics& position, const RotationSemantics& rotation)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(position.getCoordinateFrame(), rotation.getCoordinateFrame()),
                                 __PRETTY_FUNCTION__,
                                 "position and rotation expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(position.getPoint(), rotation.getOrientationFrame()),
                                 __PRETTY_FUNCTION__,
                                 "position point is different from the orientation frame origin\n")
                && reportErrorIf(!checkEqualOrUnknown(position.getReferencePoint(), rotation.getReferenceOrientationFrame()),
                                 __PRETTY_FUNCTION__,
                                 "position Ref point is different from the Ref orientation frame origin\n"));
    }

    bool TransformSemantics::setPositionSemantics(const PositionSemantics& position)
    {
        // check consistency of setted position with existing rotation
        bool status = this->check_position2rotationConsistency(position, this->getRotationSemantics());

        // set semantics
        this->positionSemantics = position;

        return status;
    }

    bool TransformSemantics::setRotationSemantics(const RotationSemantics& rotation)
    {
        // check consistency of setted position with existing rotation
        bool status = this->check_position2rotationConsistency(this->getPositionSemantics(), rotation);

        // set semantics
        this->rotationSemantics = rotation;

        return status;
    }

    TransformSemantics & TransformSemantics::operator= (const TransformSemantics & other)
    {
        return *this;
    }

    std::string TransformSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
        << " point " << this->getPositionSemantics().getPoint()
        << " orientationFrame " << this->getRotationSemantics().getOrientationFrame()
        << " referencePoint " << this->getPositionSemantics().getReferencePoint()
        << " referenceOrientationFrame " << this->getRotationSemantics().getReferenceOrientationFrame();

        return ss.str();
    }

    std::string TransformSemantics::reservedToString() const
    {
        return this->toString();
    }



}