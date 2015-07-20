/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MOTION_VECTOR_3_H
#define IDYNTREE_MOTION_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates and semantics for any motion vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe a linear or angular velocity or acceleration.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every motion vectors.
     *
     */
    template <class MotionT, class MotionAssociationsT>
    class MotionVector3: public GeomVector3<MotionT, MotionAssociationsT>
    {
    private:
        /**
         * Helper template function for computing the cross product
         */
        template <class DerivedT, class OperandT>
        struct rawOperator
        {
            DerivedT cross(const OperandT& other) const
            {
                DerivedT result;
                Eigen::Map<const Eigen::Vector3d> thisData(this->data());
                Eigen::Map<const Eigen::Vector3d> otherData(other.data());
                Eigen::Map<Eigen::Vector3d> resultData(result.data());
                
                resultData = thisData.cross(otherData);
                
                return result;
            }
        };
        
    public:
        /**
         * constructors
         */
        MotionVector3();
        MotionVector3(const double* in_data, const unsigned int in_size);
        MotionVector3(const MotionVector3 & other);
        virtual ~MotionVector3();
        
        /**
         * Types of Derivative by Linear or Angular motion vectors
         */
        typedef typename MotionAssociationsT::template DerivativeOf<LinearMotionAssociationsT>::Type MotionCrossLinM;
        typedef typename MotionAssociationsT::template DerivativeOf<AngularMotionAssociationsT>::Type MotionCrossAngM;
        typedef typename MotionAssociationsT::template DerivativeOf<LinearForceAssociationsT>::Type MotionCrossLinF;
        typedef typename MotionAssociationsT::template DerivativeOf<AngularForceAssociationsT>::Type MotionCrossAngF;
        
        /* Cross products */
        MotionCrossLinM cross(const LinearMotionVector3& other) const;
        MotionCrossAngM cross(const AngularMotionVector3& other) const;
        MotionCrossLinF cross(const LinearForceVector3& other) const;
        MotionCrossAngF cross(const AngularForceVector3& other) const;
    };
    
    /**
     * Method definitions
     */
    
    // constructors
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(): GeomVector3<MotionT, MotionAssociationsT>()
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(const double* in_data, const unsigned int in_size): GeomVector3<MotionT, MotionAssociationsT>(in_data, in_size)
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(const MotionVector3 & other): GeomVector3<MotionT, MotionAssociationsT>(other)
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::~MotionVector3()
    {}

    /* Cross products */
    template <class MotionT, class MotionAssociationsT>
    typename MotionVector3<MotionT, MotionAssociationsT>::MotionCrossLinM MotionVector3<MotionT, MotionAssociationsT>::cross(const LinearMotionVector3& other) const
    {
        return this->rawOperator<MotionCrossLinM, LinearMotionVector3>.cross(other);
    }
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionVector3<MotionT, MotionAssociationsT>::MotionCrossAngM MotionVector3<MotionT, MotionAssociationsT>::cross(const AngularMotionVector3& other) const
    {
        return this->rawOperator<MotionCrossAngM, AngularMotionVector3>.cross(other);
    }
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionVector3<MotionT, MotionAssociationsT>::MotionCrossLinF MotionVector3<MotionT, MotionAssociationsT>::cross(const LinearForceVector3& other) const
    {
        return this->rawOperator<MotionCrossLinF, LinearForceVector3>.cross(other);
    }
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionVector3<MotionT, MotionAssociationsT>::MotionCrossAngF MotionVector3<MotionT, MotionAssociationsT>::cross(const AngularForceVector3& other) const
    {
        return this->rawOperator<MotionCrossAngF, AngularForceVector3>.cross(other);
    }
}

#endif /* IDYNTREE_MOTION_VECTOR_3_H */