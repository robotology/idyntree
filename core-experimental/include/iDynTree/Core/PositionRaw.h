/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_POSITION_RAW_H
#define IDYNTREE_POSITION_RAW_H

#include <string>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates for iDynTree::Position class.
     */
    class PositionRaw
    {
    protected:
        /**
         * Storage for the Position Coordinate:
         * contains the x, y and z coordinates
         * of the position.
         */
        double privateData[3];

    public:
        /**
         * Default constructor: initialize all the coordinates to 0
         */
        PositionRaw();

        /**
         * Constructor from 3 doubles: initialize the coordinates with the passed values.
         */
        PositionRaw(double x, double y, double z);

        /**
         * Copy constructor: create a PositionRaw from another PositionRaw
         */
        PositionRaw(const PositionRaw & other);

        /**
         * Denstructor
         */
        virtual ~PositionRaw();

        /**
         * Return a coordinate by value, the index is checked
         * if it is inside 0..2 if NDEBUG is not set.
         *
         */
        const double & operator()(int index) const;

        /**
         * Return a reference to a coordinate, the index is checked
         * if it is inside 0..2 if NDEBUG is not set.
         */
        double& operator()(int index);

        /**
         * Raw data accessor: return a pointer to a vector of 3 doubles,
         * representing the x,y and z coordinates of the Position
         */
        const double * data() const;

        /**
         * Raw data accessor: return a pointer to a vector of 3 doubles,
         * representing the x,y and z coordinates of the Position
         */
        double * data();

        const PositionRaw & changePoint(const PositionRaw & newPoint);
        const PositionRaw & changeRefPoint(const PositionRaw & newRefPoint);
        static PositionRaw compose(const PositionRaw & op1, const PositionRaw & op2);
        static PositionRaw inverse(const PositionRaw & op);

        PositionRaw operator+(const PositionRaw &other) const;
        PositionRaw operator-(const PositionRaw &other) const;
        PositionRaw operator-() const;


        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        ///@}

    };
}

#endif /* IDYNTREE_POSITION_RAW_H */