/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SPATIAL_VECTOR_H
#define IDYNTREE_SPATIAL_VECTOR_H

#include "Position.h"
#include "Rotation.h"
#include "Utils.h"
#include <iDynTree/Core/GeomVector3.h>


#include <iostream>
#include <sstream>

namespace iDynTree
{
    class SpatialMotionVector;
    class SpatialForceVector;
    class Position;
    class Rotation;

    /**
 * Traits class for SpatialMotionVector and SpatialForceVector classes
 */
    template <class SpatialMotionForceVectorT>
    class SpatialMotionForceVectorT_traits {};

    template <>
    class SpatialMotionForceVectorT_traits<SpatialMotionVector>
    {
    public:
        typedef LinearMotionVector3 LinearVector3Type;
        typedef AngularMotionVector3 AngularVector3Type;
    };

    template <>
    class SpatialMotionForceVectorT_traits<SpatialForceVector>
    {
    public:
        typedef LinearForceVector3 LinearVector3Type;
        typedef AngularForceVector3 AngularVector3Type;
    };

    /**
     * Helper structure for dual space definition
     */
    template <class SpatialVectorT> struct DualSpace {};

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
#define SPATIALVECTOR_TEMPLATE_HDR \
template <typename DerivedSpatialVecT>

#define SPATIALVECTOR_INSTANCE_HDR \
SpatialVector<DerivedSpatialVecT>

    SPATIALVECTOR_TEMPLATE_HDR
    class SpatialVector
    {
    public:
        typedef typename SpatialMotionForceVectorT_traits<DerivedSpatialVecT>::LinearVector3Type LinearVector3T;
        typedef typename SpatialMotionForceVectorT_traits<DerivedSpatialVecT>::AngularVector3Type AngularVector3T;
        typedef typename DualSpace<DerivedSpatialVecT>::Type DualVectorT;

    protected:
        LinearVector3T linearVec3;
        AngularVector3T angularVec3;
        static const unsigned int linearOffset = 0;
        static const unsigned int angularOffset = 3;
        static const unsigned int totalSize = 6;

    public:
        /**
         * constructors
         */
        SpatialVector();
        SpatialVector(const LinearVector3T & _linearVec3, const AngularVector3T & _angularVec3);
        SpatialVector(const SpatialVector & other);

        /**
         * Vector accessors, getters, setters
         */
        LinearVector3T & getLinearVec3();
        AngularVector3T & getAngularVec3();
        const LinearVector3T & getLinearVec3() const;
        const AngularVector3T & getAngularVec3() const;
        void setLinearVec3(const LinearVector3T & _linearVec3);
        void setAngularVec3(const AngularVector3T & _angularVec3);

        /**
         * Vector element accessors, getters, setters
         */
        double operator()(const unsigned int index) const; // No input checking.
        double& operator()(const unsigned int index); // No input checking.
        double getVal(const unsigned int index) const; // Perform boundary checking
        bool setVal(const unsigned int index, const double new_el);  // Perform boundary checking
        unsigned int size() const;
        void zero();

        /**
         * Geometric operations
         */
        const DerivedSpatialVecT changePoint(const Position & newPoint);
        const DerivedSpatialVecT changeCoordFrame(const Rotation & newCoordFrame);
        static DerivedSpatialVecT compose(const DerivedSpatialVecT & op1, const DerivedSpatialVecT & op2);
        static DerivedSpatialVecT inverse(const DerivedSpatialVecT & op);

        /**
         * dot product
         */
        double dot(const DualVectorT & other) const;

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

        /**
         * Conversion to basic vector.
         */
        Vector6 asVector() const;

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
     * SpatialVector Method definitions
     */

    // constructors
    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector():
    linearVec3(),
    angularVec3()
    {
    }

    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector(const LinearVector3T & _linearVec3,
                                              const AngularVector3T & _angularVec3):
    linearVec3(_linearVec3),
    angularVec3(_angularVec3)
    {
    }

    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector(const SpatialVector & other):
    linearVec3(other.getLinearVec3()),
    angularVec3(other.getAngularVec3())
    {
    }

    // Vector accessors, Getters, setters
    SPATIALVECTOR_TEMPLATE_HDR
    typename SPATIALVECTOR_INSTANCE_HDR::LinearVector3T & SPATIALVECTOR_INSTANCE_HDR::getLinearVec3()
    {
        return this->linearVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    typename SPATIALVECTOR_INSTANCE_HDR::AngularVector3T & SPATIALVECTOR_INSTANCE_HDR::getAngularVec3()
    {
        return this->angularVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    const typename SPATIALVECTOR_INSTANCE_HDR::LinearVector3T & SPATIALVECTOR_INSTANCE_HDR::getLinearVec3() const
    {
        return this->linearVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    const typename SPATIALVECTOR_INSTANCE_HDR::AngularVector3T & SPATIALVECTOR_INSTANCE_HDR::getAngularVec3() const
    {
        return this->angularVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    void SPATIALVECTOR_INSTANCE_HDR::setLinearVec3(const LinearVector3T & _linearVec3)
    {
        // set linear component
        this->linearVec3 = _linearVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    void SPATIALVECTOR_INSTANCE_HDR::setAngularVec3(const AngularVector3T & _angularVec3)
    {
        // set angular component
        this->angularVec3 = _angularVec3;
    }

    // Vector element accessors, getters, setters
    SPATIALVECTOR_TEMPLATE_HDR
    double SPATIALVECTOR_INSTANCE_HDR::operator()(const unsigned int index) const
    {
        assert(index < SpatialVector::totalSize);
        return (index<SpatialVector::angularOffset? this->linearVec3(index) : this->angularVec3(index-3));
    }

    SPATIALVECTOR_TEMPLATE_HDR
    double& SPATIALVECTOR_INSTANCE_HDR::operator()(const unsigned int index)
    {
        assert(index < SpatialVector::totalSize);
        return (index<SpatialVector::angularOffset? this->linearVec3(index) : this->angularVec3(index-3));
    }

    SPATIALVECTOR_TEMPLATE_HDR
    double SPATIALVECTOR_INSTANCE_HDR::getVal(const unsigned int index) const
    {
        if( index >= SpatialVector::totalSize )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return 0.0;
        }
        return (*this)(index);
    }

    SPATIALVECTOR_TEMPLATE_HDR
    bool SPATIALVECTOR_INSTANCE_HDR::setVal(const unsigned int index, const double new_el)
    {
        if( index >= SpatialVector::totalSize )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return false;
        }
        (*this)(index) = new_el;

        return true;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    unsigned int SPATIALVECTOR_INSTANCE_HDR::size() const
    {
        return SpatialVector::totalSize;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    void SPATIALVECTOR_INSTANCE_HDR::zero()
    {
        for(unsigned int i=0; i < SpatialVector::totalSize; i++ )
        {
            (*this)(i) = 0.0;
        }
    }



    // Geometric operations
    SPATIALVECTOR_TEMPLATE_HDR
    const DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::changePoint(const Position & newPoint)
    {
        return newPoint.changePointOf(*this);
    }

    SPATIALVECTOR_TEMPLATE_HDR
    const DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::changeCoordFrame(const Rotation & newCoordFrame)
    {
        return newCoordFrame.changeCoordFrameOf(*this);
    }

    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::compose(const DerivedSpatialVecT & op1, const DerivedSpatialVecT & op2)
    {
        return DerivedSpatialVecT(op1.getLinearVec3()+op2.getLinearVec3(),
                                  op1.getAngularVec3()+op2.getAngularVec3());
    }

    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::inverse(const DerivedSpatialVecT & op)
    {
        return DerivedSpatialVecT(-op.getLinearVec3(),
                                  -op.getAngularVec3());
    }

    // dot product
    SPATIALVECTOR_TEMPLATE_HDR
    double SPATIALVECTOR_INSTANCE_HDR::dot(const DualVectorT & other) const
    {
        return (this->getLinearVec3().dot(other.getLinearVec3())
             + this->getAngularVec3().dot(other.getAngularVec3()));
    }

    // overloaded operators
    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::operator+(const DerivedSpatialVecT &other) const
    {
        return compose(*this, other);
    }

    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::operator-(const DerivedSpatialVecT &other) const
    {
        return compose(*this, inverse(other));
    }

    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::operator-() const
    {
        return inverse(*this);
    }

    // constructor helpers
    SPATIALVECTOR_TEMPLATE_HDR
    DerivedSpatialVecT SPATIALVECTOR_INSTANCE_HDR::Zero()
    {
        DerivedSpatialVecT ret;
        ret.linearVec3.zero();
        ret.angularVec3.zero();
        return ret;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    Vector6 SPATIALVECTOR_INSTANCE_HDR::asVector() const
    {
        Vector6 ret;
        ret.data()[0] = linearVec3.data()[0];
        ret.data()[1] = linearVec3.data()[1];
        ret.data()[2] = linearVec3.data()[2];
        ret.data()[3] = angularVec3.data()[0];
        ret.data()[4] = angularVec3.data()[1];
        ret.data()[5] = angularVec3.data()[2];
        return ret;
    }


    SPATIALVECTOR_TEMPLATE_HDR
    std::string SPATIALVECTOR_INSTANCE_HDR::toString() const
    {
        std::stringstream ss;

        ss << linearVec3.toString() << " "
        << angularVec3.toString();
        ss << std::endl;

        return ss.str();
    }

    SPATIALVECTOR_TEMPLATE_HDR
    std::string SPATIALVECTOR_INSTANCE_HDR::reservedToString() const
    {
        return this->toString();
    }

#undef SPATIALVECTOR_TEMPLATE_HDR
#undef SPATIALVECTOR_INSTANCE_HDR
}

#endif /* IDYNTREE_SPATIAL_VECTOR_H */
