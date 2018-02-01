/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/PositionSemantics.h>
#include <iDynTree/Core/RotationSemantics.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>
#include <iDynTree/Core/PrivatePreProcessorUtils.h>

#include <cstdio>
#include <sstream>

namespace iDynTree
{
    PositionSemantics::PositionSemantics()
    {
        iDynTreeSemanticsOp(this->setToUnknown());
    }


    PositionSemantics::PositionSemantics(int _point, int _body,
                                         int _refPoint, int _refBody,
                                         int _coordinateFrame): point(_point),
                                                                body(_body),
                                                                refPoint(_refPoint),
                                                                refBody(_refBody),
                                                                coordinateFrame(_coordinateFrame)
    {
    }


    PositionSemantics::PositionSemantics(const PositionSemantics& other)
    {
        this->point = other.point;
        this->body = other.body;
        this->refPoint = other.refPoint;
        this->refBody = other.refBody;
        this->coordinateFrame = other.coordinateFrame;
    }

    void PositionSemantics::setToUnknown()
    {
        this->point = UNKNOWN;
        this->body = UNKNOWN;
        this->refPoint = UNKNOWN;
        this->refBody = UNKNOWN;
        this->coordinateFrame = UNKNOWN;
    }

    void PositionSemantics::setPoint(int _point)
    {
        this->point = _point;
    }

    int PositionSemantics::getPoint() const
    {
        return this->point;
    }

    void PositionSemantics::setBody(int _body)
    {
        this->body = _body;
    }

    int PositionSemantics::getBody() const
    {
        return this->body;
    }

    void PositionSemantics::setReferencePoint(int _referencePoint)
    {
        this->refPoint = _referencePoint;
    }

    int PositionSemantics::getReferencePoint() const
    {
        return this->refPoint;
    }

    void PositionSemantics::setRefBody(int _refBody)
    {
        this->refBody = _refBody;
    }

    int PositionSemantics::getRefBody() const
    {
        return this->refBody;
    }

    void PositionSemantics::setCoordinateFrame(int _coordinateFrame)
    {
        this->coordinateFrame = _coordinateFrame;
    }

    int PositionSemantics::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }

    bool PositionSemantics::check_changePoint(const PositionSemantics& newPoint)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(newPoint.coordinateFrame,this->coordinateFrame),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newPoint expressed in a different coordinateFrame\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.refPoint,this->point),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newPoint has a reference point different from the original Position point\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.body,newPoint.refBody),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newPoint point and reference point are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.refBody,this->body),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newPoint reference point and original Position point are not fixed to the same body\n"));
    }

    bool PositionSemantics::changePoint(const PositionSemantics& newPoint)
    {
        // check semantics
        bool status = this->check_changePoint(newPoint);

        // set new semantics
        this->point = newPoint.getPoint();

        return status;
    }


    bool PositionSemantics::check_changeRefPoint(const PositionSemantics& newRefPoint)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(newRefPoint.coordinateFrame,this->coordinateFrame),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newRefPoint expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.point,this->refPoint),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newRefPoint point is different from the original reference point\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.body,newRefPoint.refBody),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newRefPoint point and reference point are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.body,this->refBody),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "newRefPoint point and original Position reference point are not fixed to the same body\n"));
    }

    bool PositionSemantics::changeRefPoint(const PositionSemantics& newRefPoint)
    {
        // check semantics
        bool status = this->check_changeRefPoint(newRefPoint);

        // set new semantics
        this->refPoint = newRefPoint.refPoint;

        return status;
    }

    bool PositionSemantics::check_compose(const PositionSemantics& op1, const PositionSemantics& op2)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "composing two positions expressed in different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point) && (op1.point == op2.refPoint),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "Position op1 reference point and Position op2 point don't match\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.body),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "Position op1 reference body and Position op2 body don't match\n"));
    }

    bool PositionSemantics::compose(const PositionSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        // check semantics
        bool status = PositionSemantics::check_compose(op1,op2);

        // set new semantics
        result.point = op1.point;
        result.body = op1.body;
        result.refPoint = op2.refPoint;
        result.refBody = op2.refBody;
        result.coordinateFrame = op1.coordinateFrame;

        return status;
    }

    bool PositionSemantics::check_inverse(const PositionSemantics& /*op*/)
    {
        return true;
    }

    bool PositionSemantics::inverse(const PositionSemantics& op, PositionSemantics& result)
    {
        // check semantics
        bool status = PositionSemantics::check_inverse(op);

        // set new semantics
        result.point = op.refPoint;
        result.body = op.refBody;
        result.refPoint = op.point;
        result.refBody = op.body;
        result.coordinateFrame = op.coordinateFrame;

        return status;
    }

    std::string PositionSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " point " << this->getPoint()
           << " body " << this->getBody()
           << " reference Point " << this->getReferencePoint()
           << " reference body " << this->getRefBody()
           << " coordinate Frame " << this->getCoordinateFrame();

        return ss.str();
    }

    std::string PositionSemantics::reservedToString() const
    {
        return this->toString();
    }



}
