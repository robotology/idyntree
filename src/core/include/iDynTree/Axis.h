// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_AXIS_H
#define IDYNTREE_AXIS_H

#include <string>

#include <iDynTree/Direction.h>
#include <iDynTree/Position.h>
#include <iDynTree/Utils.h>

namespace iDynTree
{
    class Transform;
    class TransformDerivative;
    class Twist;
    class SpatialAcc;
    /**
     * Class representing an axis (a directed line) in space.
     *
     * The axis is represented as a origin plus a direction.
     *
     * \ingroup iDynTreeCore
     *
     *
     */
    class Axis
    {
    private:
        Direction direction;
        Position  origin;

        /**
         * Set the object to the default axis: direction : 1, 0, 0 , point: 0, 0, 0
         */
        void setToDefault();

    public:
        /**
         * Default constructor.
         * The data is not reset to the default for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline Axis() {}

        /**
         * Constructor from a Direction and an origin, represented by a Position object.
         */
        Axis(const Direction & _direction, const Position & _origin);

        /**
         * Copy constructor: create a Axis from another Axis
         */
        Axis(const Axis & other);

        /**
         * Assignment operator: assign a Axis from another Axis
         */
        Axis& operator=(const Axis& other);

         /**
         * Get the direction of the axis
         */
        const Direction & getDirection() const;

        /**
         * Get the origin of the axis
         */
        const Position & getOrigin() const;

        /**
         * Set the direction of the axis
         */
        void setDirection(const Direction & _direction);

        /**
         * Set the origin of the axis
         */
        void setOrigin(const Position & _position);

        /**
         * Get the transform induced by a rotation of an angle theta
         * around this axis. The returned transform is the nonRotated_T_rotated,
         * such that if we have a quantity expressed in the frame obtained
         * by the rotation v_rotated, we can transform it back in the
         * non-rotated frame using the returned transform:
         * v_nonRotated = nonRotated_T_rotated*v_rotated
         */
        Transform getRotationTransform(const double theta) const;

        /**
         * Get the derivative of the getRotationTransform function with respect
         * to the theta argument.
         */
        TransformDerivative getRotationTransformDerivative(const double theta) const;

        Twist getRotationTwist(const double dtheta) const;

        SpatialAcc getRotationSpatialAcc(const double d2theta) const;

        /**
         * Get the transform induced by a translation of a distance dist
         * along this axis. The returned transform is the nonTranslated_T_translated,
         * such that if we have a quantity expressed in the frame obtained
         * by the translation v_translated, we can transform it back in the
         * non-translated frame using the returned transform:
         * v_nonTranslated = nonTranslated_T_translated*v_translated
         */
        Transform getTranslationTransform(const double dist) const;

        /**
         * Get the derivative of the getTranslationTransform function with respect
         * to the dist argument.
         */
        TransformDerivative getTranslationTransformDerivative(const double /*dist*/) const;

        Twist getTranslationTwist(const double ddist) const;

        SpatialAcc getTranslationSpatialAcc(const double d2dist) const;

        /**
         * Check if two axes are parallel (i.e. their direction are parallel).
         *
         * @param otherAxis the axes to check for parallelism.
         * @param tolerance tolerance to use in the parallelism check.
         */
        bool isParallel(const Axis & otherAxis, const double tolerance) const;

        /**
         * Return the axis with the same origin, but reversed direction.
         */
        Axis reverse() const;

        /**
         * Compute the point on the axis that is closest to a given opoint
         */
        Position getPointOnAxisClosestToGivenPoint(const iDynTree::Position& point) const;

        /**
         * Compute distance between the axis and a given point
         */
        double getDistanceBetweenAxisAndPoint(const iDynTree::Position& point) const;

        /**
         * @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}
    };
}

#endif
