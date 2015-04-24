/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <string>

namespace iDynTree
{
    /**
     * Class representation the coordinates of the Position of
     * a point with respect to another point.
     *
     * The exact semantics for this class are the one defined as PositionCoord in:
     *
     * De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
     * (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
     * IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
     * URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
     *
     */
    class Position
    {
    private:
        /**
         * Storage for the Position Coordinate:
         * contains the x, y and z coordinates
         * of the position.
         */
        double privateData[3];

        /**
         * Position semantics
         */
        int point;
        int referencePoint;
        int coordinateFrame;

        /**
         * Helper methods for actually performing operation on coordinates
         */
        const Position & changePointCoordinates(const Position & newPoint);
        const Position & changeRefPointCoordinates(const Position & newRefPoint);
        static void composeCoordinates(const Position & op1, const Position & op2, Position & result);
        static void inverseCoordinates(const iDynTree::Position& op, Position & result);

        /**
         * Helper methods for semantics of Position operation
         */
        bool changePointSemantics(const Position & newPoint);
        bool changeRefPointSemantics(const Position & newPoint);
        static bool composeSemantics(const Position & op1, const Position & op2, Position & result);
        static bool inverseSemantics(const Position & op, Position & result);


    public:
        /**
         * Default constructor: initialize all the coordinates to 0
         */
        Position();

        /**
         * Constructor from 3 doubles: initialize the coordinates with the passed values.
         */
        Position(double x, double y, double z);

        /**
         * Copy constructor: create a Position from another Position
         */
        Position(const Position & other);

        /**
         * Denstructor
         */
        ~Position();


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
         * Return a coordinate by value, the index is checked
         * if it is inside 0..2 if NDEBUG is set.
         *
         */
        const double & operator[](int index) const;

        /**
         * Return a reference to a coordinate, the index is checked
         * if it is inside 0..2 if NDEBUG is set.
         */
        double& operator[](int index);

        /**
         * Raw data accessor: return a pointer to a vector of 3 doubles,
         * representing the x,y and z coordinates of the Position
         */
        const double * data() const;

        /** @name Semantics setters and getters.
         *  Semantics setters and getters.
         */
        ///@{
        int getPoint() const;
        int getReferencePoint() const;
        int getCoordinateFrame() const;

        void setPoint(int _point);
        void setReferencePoint(int _referencePoint);
        void setCoordinateFrame(int _coordinateFrame);
        ///@}

        const Position & changePoint(const Position & newPoint);
        const Position & changeRefPoint(const Position & newRefPoint);
        static Position compose(const Position & op1, const Position & op2);
        static Position inverse(const Position & op);

        Position operator+(const Position &other) const;
        Position operator-(const Position &other) const;
        Position operator-() const;

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        ///@}

    };
}