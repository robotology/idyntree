/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ROTATION_H
#define IDYNTREE_ROTATION_H

#include <string>
#include <iDynTree/Core/RotationRaw.h>
#include <iDynTree/Core/RotationSemantics.h>

namespace iDynTree
{
    class Position;
    class Twist;
    class Wrench;
    class Direction;
    class Axis;
    class SpatialAcc;
    class SpatialMomentum;
    class ClassicalAcc;

    /**
     * Class representation the rotation of an orientation frame
     * with respect to a reference orientation frame, expressed as a Rotation matrix.
     *
     * \ingroup iDynTreeCore
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

        /**
         * Copy constructor: create a Rotation from another RotationRaw and another RotationSemantics.
         */
        Rotation(const RotationRaw & other, RotationSemantics & semantics);

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
         * Copy constructor: create a Rotation from another Rotation.
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

        /**
         * Geometric operations.
         * For the inverse2() operation, both the forward and the inverse geometric relations have to
         * be expressed in the reference orientation frame!!
         *
         */
        const Rotation & changeOrientFrame(const Rotation & newOrientFrame);
        const Rotation & changeRefOrientFrame(const Rotation & newRefOrientFrame);
        static Rotation compose(const Rotation & op1, const Rotation & op2);
        static Rotation inverse2(const Rotation & orient);
        Position changeCoordFrameOf(const Position & other) const;
        Twist  changeCoordFrameOf(const Twist & other) const;
        Wrench changeCoordFrameOf(const Wrench & other) const;
        Direction changeCoordFrameOf(const Direction & other) const;
        Axis      changeCoordFrameOf(const Axis & other) const;
        SpatialAcc  changeCoordFrameOf(const SpatialAcc & other) const;
        SpatialMomentum changeCoordFrameOf(const SpatialMomentum & other) const;
        ClassicalAcc changeCoordFrameOf(const ClassicalAcc & other) const;


        /**
          * overloaded operators
          */
        Rotation operator*(const Rotation & other) const;
        Rotation inverse() const;
        Position operator*(const Position & other) const;
        Twist    operator*(const Twist    & other) const;
        Wrench   operator*(const Wrench   & other) const;
        Direction operator*(const Direction & other) const;
        Axis      operator*(const Axis    & other) const;
        SpatialAcc      operator*(const SpatialAcc    & other) const;
        SpatialMomentum operator*(const SpatialMomentum   & other) const;
        ClassicalAcc    operator*(const ClassicalAcc    & other) const;

        /**
         * @name Initialization helpers.
         *
         */
        ///@{

        /**
         * Return a Rotation around axis X of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the X axis
         */
        static Rotation RotX(const double angle);

        /**
         * Return a Rotation around axis Y of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the Y axis
         */
        static Rotation RotY(const double angle);

        /**
         * Return a Rotation around axis Z of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the Z axis
         */
        static Rotation RotZ(const double angle);

        /**
         * Return a Rotation around axis given by direction of given angle
         *
         * @param direction the Direction around with to rotate
         * @param angle the angle (in Radians) of the rotation arount the Z axis
         */
        static Rotation RotAxis(const Direction & direction, const double angle);

        /**
         * Return a rotation object given Roll, Pitch and Yaw values.
         *
         * @note This method is compatible with the KDL::Rotation::RPY method.
         */
        static Rotation RPY(const double roll, const double pitch, const double yaw);

        /**
         * Return an identity rotation.
         *
         *
         */
        static Rotation Identity();

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

#endif