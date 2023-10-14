// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_DIRECTION_H
#define IDYNTREE_DIRECTION_H

#include <string>

#include <iDynTree/VectorFixSize.h>
#include <iDynTree/Utils.h>


namespace iDynTree
{
    /**
     * Class representing the coordinates of a direction in the 3D space
     *
     * \ingroup iDynTreeCore
     *
     */
    class Direction: public Vector3
    {
    private:
        /**
         * Set the object to the default direction : 1, 0, 0 .
         */
        void setToDefault();

    public:
        /**
         * Default constructor.
         * The data is not reset to the default for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline Direction() {}

        /**
         * Constructor from 3 doubles: initialize the direction with the passed values.
         * The vector passed is normalized to ensure that the direction is stored as a unit vector.
         */
        Direction(double x, double y, double z);

        /**
         * Copy constructor: create a Direction from another Direction
         */
        Direction(const Direction & other);

        /**
         * Copy constructor: create a Direction from a 3 double buffer
         */
        Direction(const double* in_data, const unsigned int in_size);

        /**
         * Assignment operator: assign a Direction from another Direction
         */
        Direction& operator=(const Direction& other);

        /**
         * Normalize the representation of the direction, useful if
         * the coordinates of the direction has been manually setted
         * and you want to be sure that this direction is actually
         * a unit vector.
         *
         * @param tol if the norm of the vector < tol, set the direction to 1,0,0
         */
        void Normalize(double tol=DEFAULT_TOL);

        /**
         * Check if two directions are parallel.
         *
         * @param otherDirection the direction to check for parallelism.
         * @param tolerance tolerance to use in the parallelism check.
         */
        bool isParallel(const Direction & otherDirection, double tolerance) const;

        /**
         * Check if two directions are perpendicular.
         *
         * @param otherDirection the direction to check for the perpendicular check.
         * @param tolerance tolerance to use in the perpendicular check.
         */
        bool isPerpendicular(const Direction & otherDirection, double tolerance) const;

        /**
         * Return the direction, i.e. return its opposite.
         *
         */
        Direction reverse() const;

        /**
         * @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

        static Direction Default();
    };
}

#endif
