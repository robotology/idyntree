/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_POSITION_RAW_H
#define IDYNTREE_POSITION_RAW_H

#include "IVector.h"
#include <string>

namespace iDynTree
{
    class RotationRaw;
    
    /**
     * Class providing the raw coordinates for iDynTree::Position class.
     *
     * \ingroup iDynTreeCore
     */
    class PositionRaw: public IVector
    {
    protected:
        /**
         * Storage for the Position Coordinate:
         * contains the x, y and z coordinates
         * of the position.
         */
        double m_data[3];
        
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
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to PositionRaw.
         */
        ///@{
        double operator()(const unsigned int index) const;
        
        double& operator()(const unsigned int index);
        
        double getVal(const unsigned int index) const;
        
        bool setVal(const unsigned int index, const double new_el);
        
        unsigned int size() const;
        
        ///@}
        
        
        
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
        
        /**
         * Assign all element of the vector to 0.
         */
        void zero();
        
        
        const PositionRaw & changePoint(const PositionRaw & newPoint);
        const PositionRaw & changeRefPoint(const PositionRaw & newRefPoint);
        const PositionRaw & changeCoordinateFrame(const RotationRaw & newCoordinateFrame);
        static PositionRaw compose(const PositionRaw & op1, const PositionRaw & op2);
        static PositionRaw inverse(const PositionRaw & op);
        
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        
        std::string reservedToString() const;
        ///@}
        
    };
    
}

#endif /* IDYNTREE_POSITION_RAW_H */