/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
        int refOrientationFrame;

    public:
        /**
         * Default constructor: initialize all semantics to unknown;
         */
        RotationSemantics();

        /**
         * Constructor for initializing semantics
         *
         */
        RotationSemantics(int _orientationFrame, int _refOrientationFrame);

        /**
         * Copy constructor: create a RotationSemantics from another RotationSemantics
         */
        RotationSemantics(const RotationSemantics & other);

        /**
         * Denstructor
         */
        ~RotationSemantics();

         /**
          * @name Semantics setters and getters.
          *  Semantics setters and getters.
          */
         ///@{
         int getOrientationFrame() const;
         int getReferenceOrientationFrame() const;

         void setOrientationFrame(int _orientationFrame);
         void setReferenceOrientationFrame(int _refOrientationFrame);
        ///@}

        /**
         * @name Semantics checkers
         * Check if a given operation is possible or not, given the semantics of the operators.
         */
        ///@{
        bool check_changeOrientFrame(const RotationSemantics & newOrientFrame);
        bool check_changeRefOrientFrame(const RotationSemantics & newRefOrientFrame);
        static bool check_compose(const RotationSemantics & op1, const RotationSemantics & op2);
        static bool check_inverse2(const RotationSemantics & op);
        static bool check_transform(const RotationSemantics & op1, const PositionSemantics & op2);
        ///@}

        /**
         * @name Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        ///@{
        const RotationSemantics & changeOrientFrame(const RotationSemantics & newOrientFrame);
        const RotationSemantics & changeRefOrientFrame(const RotationSemantics & newRefOrientFrame);

        static RotationSemantics compose(const RotationSemantics & op1, const RotationSemantics & op2);
        static void compose(const RotationSemantics & op1, const RotationSemantics & op2, RotationSemantics & result);
        static RotationSemantics inverse2(const RotationSemantics & op);
        static void inverse2(const RotationSemantics & op, RotationSemantics & result);
        static PositionSemantics transform(const RotationSemantics & op1, const PositionSemantics & op2);
        static void transform(const RotationSemantics & op, const PositionSemantics & op2, PositionSemantics & result);
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