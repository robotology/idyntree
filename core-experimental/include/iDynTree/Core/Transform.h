/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TRANSFORM_H
#define IDYNTREE_TRANSFORM_H

#include <string>
#include "TransformRaw.h"
#include "TransformSemantics.h"

namespace iDynTree
{
    class Position;
    class Rotation;

    /**
     * Class representation the relative displacement between two different frames.
     *
     * The semantics for this class are lousely on the one of PoseCoord in:
     *
     * De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
     * (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
     * IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
     * URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
     *
     * However this class is designed to be an easy to use proxy to perform change of frame of
     * expression for iDynTree::Position, iDynTree::Twist, iDynTree::Wrench. Hence with respect to
     * the PoseCoord we restrict the semantics to the one of the homogeneous transformation matrix
     * representation, enforcing that the reference orientation frame is always coincident with teh
     * coordinate frame . For this reason the class is called "Transform", because it will be mainly
     * used to transform quantities between frames.
     *
     * Given that this class it may used to represent homogeneous transform matrix as well as adjoint
     * matrix, no raw access to the underline storage ( data() method ) is provided, because it does not
     * have a canonical representation.
     */
    class Transform: public TransformRaw
    {
    private:
        TransformSemantics semantics;

    public:
        /**
         * Default constructor: initialize the rotation to the identity and the translation to 0
         */
        Transform();

        /**
         * Constructor from a rotation and a point (can raise error on semantics of the passed elements)
         */
        Transform(const Rotation & rot, const Position & origin);

        /**
         * Copy constructor: create a Transform from a TransformRaw.
         */
        Transform(const TransformRaw & other);

        /**
         * Copy constructor: create a Transform from another Transform.
         */
        Transform(const Transform & other);

        /**
         * Denstructor
         */
        virtual ~Transform();

        /**
         * Semantic accessor
         */
        TransformSemantics& getSemantics();

        /**
         * Const semantic getter
         */
        const TransformSemantics& getSemantics() const;

        /**
         * Get the rotation part of the transform
         */
        const Rotation getRotation() const;

        /**
         * Get the translation part of the transform
         */
        const Position getPosition() const;

        /**
         * Set the rotation part of the transform
         */
        const bool setRotation(const Rotation & rotation);

        /**
         * Set the translation part of the transform
         */
        const bool setPosition(const Position & position);

        // semantics operation
        static Transform compose(const Transform & op1, const Transform & op2);
        static Transform inverse2(const Transform & orient);
        static Position apply(const Transform & op1, const Position & op2);

        /** overloaded operators **/
        Transform operator*(const Transform & other) const;
        Transform inverse() const;
        Position operator*(const Position & op2) const;

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        ///@}
    };
}

#endif