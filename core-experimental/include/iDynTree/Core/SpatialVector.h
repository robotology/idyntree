/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_VECTOR_H
#define IDYNTREE_SPATIAL_VECTOR_H

#include "Position.h"
#include "Rotation.h"

namespace iDynTree
{
    class SpacialMotionVector;
    class SpacialForceVector;
    class Position;
    class Rotation;

    /**
     * Helper structure for dual space definition
     */
    template <typename SpatialVectorT> struct DualSpace {};
    
    template <> struct DualSpace<SpatialMotionVector> {typedef SpatialForceVector Type;};
    
    template <> struct DualSpace<SpatialForceVector> {typedef SpatialMotionVector Type;};

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
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
#define CLASS_TEMPLATE_HDR \
    template <typename DerivedSpatialVecT, typename LinearVector3T, typename AngularVector3T>

#define CLASS_FUNC_HDR \
    SpatialVector<DerivedSpatialVecT, LinearVector3T, AngularVector3T>
    
    CLASS_TEMPLATE_HDR
    class SpatialVector
    {
    protected:
        LinearVector3T linearVec3;
        AngularVector3T angularVec3;
//        SpatialVectorSemantics semantics;

    public:
        /**
         * constructors
         */
        SpatialVector();
        SpatialVector(const LinearVector3T & _linearVec3, const AngularVector3T & _angularVec3);
        SpatialVector(const SpatialVector & other);
        virtual ~SpatialVector();
        
        /**
         * Accessors, Getters, setters
         */
        LinearVector3T & getLinearVec3();
        AngularVector3T & getAngularVec3();
        const LinearVector3T & getLinearVec3() const;
        const AngularVector3T & getAngularVec3() const;
        void setLinearVec3(const LinearVector3T & _linearVec3);
        void setAngularVec3(const AngularVector3T & _angularVec3);

        /**
         * Geometric operations
         */
        const DerivedSpatialVecT & changePoint(const Position & newPoint);
        const DerivedSpatialVecT & changeCoordFrame(const Rotation & newCoordFrame);
        static DerivedSpatialVecT compose(const DerivedSpatialVecT & op1, const DerivedSpatialVecT & op2);
        static DerivedSpatialVecT inverse(const DerivedSpatialVecT & op);
        
        /**
         * dot product
         */
        double dot(const typename DualSpace<DerivedSpatialVecT>::Type & other) const;
        
        /**
         * overloaded operators
         */
        DerivedSpatialVecT operator+(const DerivedSpatialVecT &other) const;
        DerivedSpatialVecT operator-(const DerivedSpatialVecT &other) const;
        DerivedSpatialVecT operator-() const;
        
        /**
         * constructor helpers
         */
        static DerivedSpatialVecT Zero();
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        
        std::string reservedToString() const;
        ///@}
    };


    /**
     *====================================================================================
     * Method definitions
     */
    
    // constructors
    CLASS_TEMPLATE_HDR
    CLASS_FUNC_HDR::SpatialVector(): linearVec3(),
                                     angularVec3()
    {}
    
    CLASS_TEMPLATE_HDR
    CLASS_FUNC_HDR::SpatialVector(const LinearVector3T & _linearVec3,
                                  const AngularVector3T & _angularVec3): linearVec3(_linearVec3),
                                                                         angularVec3(_angularVec3)
    {}
    
    CLASS_TEMPLATE_HDR
    CLASS_FUNC_HDR::SpatialVector(const SpatialVector & other): linearVec3(this->getLinearVec3()),
                                                                angularVec3(this->getAngularVec3())
    {}
    
    CLASS_TEMPLATE_HDR
    CLASS_FUNC_HDR::~SpatialVector()
    {}
    
    // Accessors, Getters, setters
    CLASS_TEMPLATE_HDR
    LinearVector3T & CLASS_FUNC_HDR::getLinearVec3()
    {
        return this->linearVec3;
    }
    
    CLASS_TEMPLATE_HDR
    AngularVector3T & CLASS_FUNC_HDR::getAngularVec3()
    {
        return this->angularVec3;
    }
    
    CLASS_TEMPLATE_HDR
    const LinearVector3T & CLASS_FUNC_HDR::getLinearVec3() const
    {
        return this->linearVec3;
    }
    
    CLASS_TEMPLATE_HDR
    const AngularVector3T & CLASS_FUNC_HDR::getAngularVec3() const
    {
        return this->angularVec3;
    }
    
    CLASS_TEMPLATE_HDR
    void CLASS_FUNC_HDR::setLinearVec3(const LinearVector3T & _linearVec3)
    {
        this->linearVec3 = _linearVec3;
    }
    
    CLASS_TEMPLATE_HDR
    void CLASS_FUNC_HDR::setAngularVec3(const AngularVector3T & _angularVec3)
    {
        this->angularVec3 = _angularVec3;
    }

    // Geometric operations
    CLASS_TEMPLATE_HDR
    const DerivedSpatialVecT & CLASS_FUNC_HDR::changePoint(const Position & newPoint)
    {
        return newPoint.changePointOf(*this);
    }

    CLASS_TEMPLATE_HDR
    const DerivedSpatialVecT & CLASS_FUNC_HDR::changeCoordFrame(const Rotation & newCoordFrame)
    {
        return newCoordFrame.changeCoordFrameOf(*this);
    }
    
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::compose(const DerivedSpatialVecT & op1, const DerivedSpatialVecT & op2)
    {
        return DerivedSpatialVecT(op1.getLinearVec3()+op2.getLinearVec3(),
                                  op1.getAngularVec3()+op2.getAngularVec3());
    }
    
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::inverse(const DerivedSpatialVecT & op)
    {
        return DerivedSpatialVecT(-op.getLinearVec3(),
                                  -op.getAngularVec3());
    }
    
    // dot product
    CLASS_TEMPLATE_HDR
    double CLASS_FUNC_HDR::dot(const typename DualSpace<DerivedSpatialVecT>::Type & other) const
    {
        return (this->getLinearVec3().dot(other.getLinearVec3())
             + this->getAngularVec3().dot(other.getAngularVec3()));
    }
    
    // overloaded operators
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::operator+(const DerivedSpatialVecT &other) const
    {
        return compose(*this, other);
    }
    
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::operator-(const DerivedSpatialVecT &other) const
    {
        return compose(*this, inverse(other));
    }
    
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::operator-() const
    {
        return inverse(*this);
    }

    // constructor helpers
    CLASS_TEMPLATE_HDR
    DerivedSpatialVecT CLASS_FUNC_HDR::Zero()
    {
        return DerivedSpatialVecT();
    }

    CLASS_TEMPLATE_HDR
    std::string CLASS_FUNC_HDR::toString() const
    {
        std::stringstream ss;
        
        ss << linearVec3.toString() << " "
        << angularVec3.toString() << " "
        << /*semantics.toString() <<*/ std::endl;
        
        return ss.str();
    }

    CLASS_TEMPLATE_HDR
    std::string CLASS_FUNC_HDR::reservedToString() const
    {
        return this->toString();
    }

#undef CLASS_TEMPLATE_HDR
#undef CLASS_FUNC_HDR
}

#endif /* IDYNTREE_SPATIAL_VECTOR_H */