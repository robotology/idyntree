/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_SPATIAL_VECTOR_H
#define IDYNTREE_I_SPATIAL_VECTOR_H


namespace iDynTree
{
    /**
     * Class providing an interface to any spatial motion or force vector, which provides
     * raw coordinates.
     *
     * \ingroup iDynTreeCore
     *
     * A motion spatial vector can be used to describe twist, spatial acceleration,
     * and derivatives.
     *
     * A force spatial vector can be used to describe spatial momentum, wrench,
     * and derivatives.
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * TransformRaw::apply(SpatialForceRaw),
     * for consistency with the iDynTree::PositionRaw class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class ISpatialVector
    {
    public:
        /**
         * constructors
         */
        virtual ~SpatialMotionVector() = 0;
        
        /**
         * Geometric operations
         */
        virtual const ISpatialVector & changePoint(const PositionRaw & newPoint) = 0;
        virtual const ISpatialVector & changeCoordFrame(const RotationRaw & newCoordFrame) = 0;
        virtual static ISpatialVector compose(const ISpatialVector & op1, const ISpatialVector & op2) = 0;
        virtual static ISpatialVector inverse(const ISpatialVector & op) = 0;
        
        virtual double dot(const ISpatialVector& other) const = 0;
        
        /**
         * Cross products
         */
        virtual ISpatialVector cross(const ISpatialVector& other) const = 0;
        virtual ISpatialVector cross(const ISpatialVector& other) const = 0;
        
        /**
         * overloaded operators
         */
        Twist operator+(const Twist &other) const = 0;
        Twist operator-(const Twist &other) const = 0;
        Twist operator-() const = 0;
        
        /**
         * constructor helpers
         */
        static ISpatialVector Zero() = 0;
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        
        std::string reservedToString() const;
        ///@}
    };
}

#endif /* IDYNTREE_I_SPATIAL_VECTOR_H */