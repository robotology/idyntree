/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ROTATION_SEMANTICS_H
#define IDYNTREE_ROTATION_SEMANTICS_H

#include <string>

namespace iDynTree
{
    class PositionSemantics;

    /**
     * Class providing the semantics for iDynTree::Rotation class.
     *
     * \ingroup iDynTreeCore
     */
    class RotationSemantics
    {
    protected:
        int orientationFrame;
        int body;
        int refOrientationFrame;
        int refBody;
        int coordinateFrame;

        /**
         * @name Semantics checkers
         * Check if a given operation is possible or not, given the semantics of the operators.
         */
        ///@{
        bool check_changeOrientFrame(const RotationSemantics & newOrientFrame);
        bool check_changeRefOrientFrame(const RotationSemantics & newRefOrientFrame);
        bool check_changeCoordFrameOf(const PositionSemantics & op) const;
        static bool check_compose(const RotationSemantics & op1, const RotationSemantics & op2);
        static bool check_inverse2(const RotationSemantics & op);
        ///@}

    public:
        /**
         * Default constructor: initialize all semantics
         * to unknown if the semantic support is enabled.
         */
        RotationSemantics();

        /**
         * Constructor: initialize semantics from individual parameters
         *
         */
        RotationSemantics(int _orientationFrame, int _body, int _refOrientationFrame, int _refBody);

        /**
         * Copy constructor: create a RotationSemantics from another RotationSemantics
         */
        RotationSemantics(const RotationSemantics & other);

        void setToUnknown();

        /**
         * @name Semantics setters and getters.
         *  Semantics setters and getters.
         */
        ///@{
        int getOrientationFrame() const;
        int getBody() const;
        int getReferenceOrientationFrame() const;
        int getRefBody() const;
        int getCoordinateFrame() const;

        void setOrientationFrame(int _orientationFrame);
        void setBody(int _body);
        void setReferenceOrientationFrame(int _refOrientationFrame);
        void setRefBody(int _refBody);
        void setCoordinateFrame(int _coordinateFrame);
    ///@}

        /**
         * @name Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        ///@{
        bool changeOrientFrame(const RotationSemantics & newOrientFrame);
        bool changeRefOrientFrame(const RotationSemantics & newRefOrientFrame);
        bool changeCoordFrameOf(const PositionSemantics & other, PositionSemantics & result) const;
        static bool compose(const RotationSemantics & op1, const RotationSemantics & op2, RotationSemantics & result);
        static bool inverse2(const RotationSemantics & op, RotationSemantics & result);
        ///@}



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
