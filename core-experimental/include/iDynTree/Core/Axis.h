/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_AXIS_H
#define IDYNTREE_AXIS_H

#include <string>

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Utils.h>


namespace iDynTree
{
    /**
     * Class representing an axis (a directed line) in space.
     *
     * The axis is represented as a origin plus a direction.
     *
     * \ingroup iDynTreeCore
     *
     * This class is currently lacking semantics.
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
         * Default constructor: initialize the axis to the default, as in setToDefault
         */
        Axis();

        /**
         * Constructor from a Direction and an origin, represented by a Position object.
         */
        Axis(const Direction & _direction, const Position & _origin);

        /**
         * Copy constructor: create a Axis from another Axis
         */
        Axis(const Axis & other);

        /**
         * Destructor
         */
        virtual ~Axis();

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