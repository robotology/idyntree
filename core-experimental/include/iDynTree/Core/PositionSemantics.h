/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
        int refPoint;
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
         * Default constructor: initialize all semantics to unknown;
         */
        PositionSemantics();

        /**
         * Constructor for initializing semantics
         *
         */
        PositionSemantics(int _point, int _refPoint, int _coordinateFrame);

        /**
         * Copy constructor: create a PositionSemantics from another PositionSemantics
         */
        PositionSemantics(const PositionSemantics & other);

        /**
         * Denstructor
         */
        ~PositionSemantics();

         /**
          * @name Semantics setters and getters.
          *  Semantics setters and getters.
          */
         ///@{
         int getPoint() const;
         int getReferencePoint() const;
         int getCoordinateFrame() const;

         void setPoint(int _point);
         void setReferencePoint(int _referencePoint);
         void setCoordinateFrame(int _coordinateFrame);
        ///@}

        bool changePoint(const PositionSemantics & newPoint);
        bool changeRefPoint(const PositionSemantics & newRefPoint);
        bool changeCoordinateFrame(const RotationSemantics & newCoordinateFrame);
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