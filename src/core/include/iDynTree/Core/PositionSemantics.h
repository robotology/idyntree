/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_POSITION_SEMANTICS_H
#define IDYNTREE_POSITION_SEMANTICS_H

#include <string>

namespace iDynTree
{
    class RotationSemantics;
    /**
     * Class providing the semantics for iDynTree::Position class.
     *
     * \ingroup iDynTreeCore
     */
    class PositionSemantics
    {
    protected:
        int point;
        int body;
        int refPoint;
        int refBody;
        int coordinateFrame;

        /**
         * @name Semantics checkers
         * Check if a given operation is possible or not, given the semantics of the operators.
         */
        ///@{
        bool check_changePoint(const PositionSemantics & newPoint);
        bool check_changeRefPoint(const PositionSemantics & newRefPoint);
        bool check_changeCoordinateFrame(const RotationSemantics & newCoordinateFrame);
        static bool check_compose(const PositionSemantics & op1, const PositionSemantics & op2);
        static bool check_inverse(const PositionSemantics & op);
        ///@}


    public:
        /**
         * Default constructor: initialize all semantics to unknown if the semantics is enabled.
         */
        PositionSemantics();

        /**
         * Constructor for initializing semantics
         *
         */
        PositionSemantics(int _point, int _body, int _refPoint, int _refBody, int _coordinateFrame);

        /**
         * Copy constructor: create a PositionSemantics from another PositionSemantics
         */
        PositionSemantics(const PositionSemantics & other);

        /**
         * Constructor helper: set all the info in this semantics class to UNKNOWN .
         */
        void setToUnknown();

        /**
         * @name Semantics setters and getters.
         *  Semantics setters and getters.
         */
        ///@{
        int getPoint() const;
        int getBody() const;
        int getReferencePoint() const;
        int getRefBody() const;
        int getCoordinateFrame() const;

        void setPoint(int _point);
        void setBody(int _body);
        void setReferencePoint(int _referencePoint);
        void setRefBody(int _refBody);
        void setCoordinateFrame(int _coordinateFrame);
        ///@}

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        bool changePoint(const PositionSemantics & newPoint);
        bool changeRefPoint(const PositionSemantics & newRefPoint);
        static bool compose(const PositionSemantics & op1, const PositionSemantics & op2, PositionSemantics & result);
        static bool inverse(const PositionSemantics & op, PositionSemantics & result);

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };
}

#endif /* IDYNTREE_POSITION_SEMANTICS_H */
