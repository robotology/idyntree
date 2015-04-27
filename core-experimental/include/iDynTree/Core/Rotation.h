/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ROTATION_H
#define IDYNTREE_ROTATION_H

#include <string>
#include "RotationRaw.h"
#include "RotationSemantics.h"

namespace iDynTree
{
    class Position;

    /**
     * Class representation the rotation of an orientation frame
     * with respect to a reference orientation frame, expressed as a Rotation matrix.
     *
     * The semantics for this class is based on the OrientationCoord in:
     *
     * De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
     * (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
     * IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
     * URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
     *
     * Given that this class uses the rotation matrix to represent orientation, some operation
     * are disable because there is a semantic constraint induced by choice of representation, i.e.
     * that the coordinate frame is always the reference orientation frame. Thus, some semantic operation
     * are not enabled, namely:
     *  * the generic inverse, that does not change the coordinate frame.
     *  * changeCoordFrame, because CoordFrame is always the same of RefOrientFrame.
     */
    class Rotation: public RotationRaw
    {
    private:
        RotationSemantics semantics;

    public:
        /**
         * Default constructor: initialize all the rotation to the identity
         */
        Rotation();

        /**
         * Constructor from 9 doubles: initialize elements of the rotation matrix.
         */
        Rotation(double xx, double xy, double xz,
                 double yx, double yy, double yz,
                 double zx, double zy, double zz);

        /**
         * Copy constructor: create a Rotation from another RotationRaw.
         */
        Rotation(const RotationRaw & other);

        /**
         * Copy constructor: create a Rotation from another RotationRaw.
         */
        Rotation(const Rotation & other);

        /**
         * Denstructor
         */
        virtual ~Rotation();

        /**
         * Semantic getter
         */
        RotationSemantics& getSemantics();

        /**
         * Semantic getter
         */
        const RotationSemantics& getSemantics() const;

        const Rotation & changeOrientFrame(const Rotation & newOrientFrame);
        const Rotation & changeRefOrientFrame(const Rotation & newRefOrientFrame);
        static Rotation compose(const Rotation & op1, const Rotation & op2);
        static Rotation inverse2(const Rotation & orient);
        static Position apply(const Rotation & op1, const Position & op2);

        /** overloaded operators **/
        Rotation operator*(const Rotation & other) const;
        Rotation inverse() const;
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