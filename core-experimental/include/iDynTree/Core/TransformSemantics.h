/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TRANSFORM_SEMANTICS_H
#define IDYNTREE_TRANSFORM_SEMANTICS_H

#include <string>

namespace iDynTree
{
    class PositionSemantics;
    class RotationSemantics;

    /**
     * Class providing the semantics for iDynTree::Transform class.
     *
     * \ingroup iDynTreeCore
     */
    class TransformSemantics
    {
    private:
        int point;
        int orientationFrame;
        int refPoint;
        int refOrientationFrame;

    public:
        /**
         * Default constructor: initialy all semantics to unknown
         */
        TransformSemantics();

        /**
         * Copy constructor: create a TransformSemantics from a TransformSemantics.
         */
        TransformSemantics(const TransformSemantics & other);


        /**
         * Denstructor
         */
        virtual ~TransformSemantics();


        /**
         * Get the rotation part of the transform
         */
        const RotationSemantics getRotationSemantics() const;

        /**
         * Get the translation part of the transform
         */
        const PositionSemantics getPositionSemantics() const;

        /**
         * Set the rotation part of the transform
         */
        bool setRotationSemantics(const RotationSemantics & rotation);

        /**
         * Set the translation part of the transform
         */
        bool setPositionSemantics(const PositionSemantics & position);

        /**
          * @name Semantics setters and getters.
          *  Semantics setters and getters.
          */
        ///@{
        int getPoint() const;
        int getOrientationFrame() const;
        int getReferencePoint() const;
        int getReferenceOrientationFrame() const;

        void setPoint(int _point);
        void setOrientationFrame(int _orientationFrame);
        void setReferencePoint(int _refPoint);
        void setReferenceOrientationFrame(int _refOrientationFrame);
        ///@}

        // semantics operation
        static bool check_compose(const TransformSemantics & op1, const TransformSemantics & op2);
        static bool check_inverse2(const TransformSemantics & orient);
        static bool check_transform(const TransformSemantics & op1, const PositionSemantics & op2);

        static TransformSemantics compose(const TransformSemantics & op1, const TransformSemantics & op2);
        static void compose(const TransformSemantics & op1, const TransformSemantics & op2, TransformSemantics & result);
        static TransformSemantics inverse2(const TransformSemantics & trans);
        static void inverse2(const TransformSemantics & trans, TransformSemantics & result);
        static PositionSemantics transform(const TransformSemantics & op1, const PositionSemantics & op2);
        static void transform(const TransformSemantics & op1, const PositionSemantics & op2, PositionSemantics & result);

        /** overloaded operators **/
        TransformSemantics operator*(const TransformSemantics & other) const;
        TransformSemantics inverse() const;
        PositionSemantics operator*(const PositionSemantics & op2) const;

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}
    };
}

#endif