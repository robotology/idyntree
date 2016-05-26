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
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>
#include "Utils.h"
#include <iDynTree/Core/PrivatePreProcessorUtils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>


#include <iostream>
#include <sstream>

namespace iDynTree
{
    class SpatialMotionVector;
    class SpatialForceVector;
    class Position;
    class Rotation;

#define SPATIALVECTORSEMANTICS_TEMPLATE_HDR \
template <typename LinearVec3SemanticsT, typename AngularVec3SemanticsT>
#define SPATIALVECTORSEMANTICS_INSTANCE_HDR \
SpatialVectorSemantics<LinearVec3SemanticsT, AngularVec3SemanticsT>

    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    class SpatialVectorSemantics
    {
    protected:
        LinearVec3SemanticsT & linearVec3Semantics;
        AngularVec3SemanticsT & angularVec3Semantics;

    public:
        /**
         * constructors
         */
        SpatialVectorSemantics(LinearVec3SemanticsT & linearVec3, AngularVec3SemanticsT & angularVec3);

        bool check_linear2angularConsistency(const LinearVec3SemanticsT & linearVec3, const AngularVec3SemanticsT & angularVec3);

        /**
         * copy assignment operator
         * We redefine this operator because the compiler is unable to generate a default
         * one, since TransformSemantics is only composed by references.
         */
        SpatialVectorSemantics & operator=(const SpatialVectorSemantics & other);

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };

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

    protected:
        LinearVector3T linearVec3;
        AngularVector3T angularVec3;
        SpatialVectorSemantics<typename LinearVector3T::SemanticsType, typename AngularVector3T::SemanticsType> semantics;
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
     * SpatialVectorSemantics Method definitions
     */

    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    SPATIALVECTORSEMANTICS_INSTANCE_HDR::SpatialVectorSemantics(LinearVec3SemanticsT & linearVec3,
                                                                AngularVec3SemanticsT & angularVec3):
    linearVec3Semantics(linearVec3),
    angularVec3Semantics(angularVec3)
    {
    }

    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    SPATIALVECTORSEMANTICS_INSTANCE_HDR & SPATIALVECTORSEMANTICS_INSTANCE_HDR::operator=(const SpatialVectorSemantics & other)
    {
        return *this;
    }



    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    bool SPATIALVECTORSEMANTICS_INSTANCE_HDR::check_linear2angularConsistency(const LinearVec3SemanticsT & linearVec3,
                                                                              const AngularVec3SemanticsT & angularVec3)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(linearVec3.getBody(), angularVec3.getBody()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "linear and angular vectors are defined for different bodies\n")
                && reportErrorIf(!checkEqualOrUnknown(linearVec3.getRefBody(), angularVec3.getRefBody()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "linear and angular vectors have different reference bodies\n")
                && reportErrorIf(!checkEqualOrUnknown(linearVec3.getCoordinateFrame(), angularVec3.getCoordinateFrame()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "linear and angular vectors are expressed in different coordinateFrames\n"));
    }

    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    std::string SPATIALVECTORSEMANTICS_INSTANCE_HDR::toString() const
    {
        // \todo
        return std::string();
    }

    SPATIALVECTORSEMANTICS_TEMPLATE_HDR
    std::string SPATIALVECTORSEMANTICS_INSTANCE_HDR::reservedToString() const
    {
        return this->toString();
    }


    /**
     *====================================================================================
     * SpatialVector Method definitions
     */

    // constructors
    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector():
    linearVec3(),
    angularVec3(),
    semantics(linearVec3.semantics, angularVec3.semantics)
    {
    }

    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector(const LinearVector3T & _linearVec3,
                                              const AngularVector3T & _angularVec3):
    linearVec3(_linearVec3),
    angularVec3(_angularVec3),
    semantics(linearVec3.semantics, angularVec3.semantics)
    {
    }

    SPATIALVECTOR_TEMPLATE_HDR
    SPATIALVECTOR_INSTANCE_HDR::SpatialVector(const SpatialVector & other):
    linearVec3(other.getLinearVec3()),
    angularVec3(other.getAngularVec3()),
    semantics(linearVec3.semantics, angularVec3.semantics)
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
        // check semantics
        iDynTreeAssert(semantics.check_linear2angularConsistency(_linearVec3.semantics,
                                                                 this->angularVec3.semantics));
        // set linear component
        this->linearVec3 = _linearVec3;
    }

    SPATIALVECTOR_TEMPLATE_HDR
    void SPATIALVECTOR_INSTANCE_HDR::setAngularVec3(const AngularVector3T & _angularVec3)
    {
        // check semantics
        iDynTreeAssert(semantics.check_linear2angularConsistency(this->linearVec3.semantics,
                                                                 _angularVec3.semantics));
        // set angular component
        this->angularVec3 = _angularVec3;
    }

    // Vector element accessors, getters, setters
    SPATIALVECTOR_TEMPLATE_HDR
    double SPATIALVECTOR_INSTANCE_HDR::operator()(const unsigned int index) const
    {
        return (index<SpatialVector::angularOffset? this->linearVec3(index) : this->angularVec3(index-3));
    }

    SPATIALVECTOR_TEMPLATE_HDR
    double& SPATIALVECTOR_INSTANCE_HDR::operator()(const unsigned int index)
    {
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
    double SPATIALVECTOR_INSTANCE_HDR::dot(const typename DualSpace<DerivedSpatialVecT>::Type & other) const
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
        iDynTreeSemanticsOp(ss << " " << semantics.toString());
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
