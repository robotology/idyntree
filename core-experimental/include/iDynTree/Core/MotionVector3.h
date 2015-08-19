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

#define MOTIONVECTOR3_TEMPLATE_HDR \
template <class MotionT, class MotionAssociationsT, class MotionTSemantics>
#define MOTIONVECTOR3_INSTANCE_HDR \
MotionVector3<MotionT, MotionAssociationsT, MotionTSemantics>

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
    MOTIONVECTOR3_TEMPLATE_HDR
    class MotionVector3: public GeomVector3<MotionT, MotionAssociationsT, MotionTSemantics>
    {
    private:
        /**
         * Helper template function for computing the cross product
         */
        template <class DerivedT, class OperandT>
        struct rawOperator
        {
            static DerivedT cross(const MotionVector3<MotionT, MotionAssociationsT, MotionTSemantics>& _this, const OperandT& other)
            {
                DerivedT result;
                Eigen::Map<const Eigen::Vector3d> thisData(_this.data());
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
    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::MotionVector3():
    GeomVector3<MotionT, MotionAssociationsT, MotionTSemantics>()
    {}
    
    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::MotionVector3(const double* in_data, const unsigned int in_size):
    GeomVector3<MotionT, MotionAssociationsT, MotionTSemantics>(in_data, in_size)
    {}
    
    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::MotionVector3(const MotionVector3 & other):
    GeomVector3<MotionT, MotionAssociationsT, MotionTSemantics>(other)
    {}
    
    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::~MotionVector3()
    {}

    /* Cross products */
    MOTIONVECTOR3_TEMPLATE_HDR
    typename MOTIONVECTOR3_INSTANCE_HDR::MotionCrossLinM MOTIONVECTOR3_INSTANCE_HDR::cross(const LinearMotionVector3& other) const
    {
        return rawOperator<MotionCrossLinM, LinearMotionVector3>::cross(*this, other);
    }
    
    MOTIONVECTOR3_TEMPLATE_HDR
    typename MOTIONVECTOR3_INSTANCE_HDR::MotionCrossAngM MOTIONVECTOR3_INSTANCE_HDR::cross(const AngularMotionVector3& other) const
    {
        return rawOperator<MotionCrossAngM, AngularMotionVector3>::cross(*this, other);
    }
    
    MOTIONVECTOR3_TEMPLATE_HDR
    typename MOTIONVECTOR3_INSTANCE_HDR::MotionCrossLinF MOTIONVECTOR3_INSTANCE_HDR::cross(const LinearForceVector3& other) const
    {
        return rawOperator<MotionCrossLinF, LinearForceVector3>::cross(*this, other);
    }
    
    MOTIONVECTOR3_TEMPLATE_HDR
    typename MOTIONVECTOR3_INSTANCE_HDR::MotionCrossAngF MOTIONVECTOR3_INSTANCE_HDR::cross(const AngularForceVector3& other) const
    {
        return rawOperator<MotionCrossAngF, AngularForceVector3>::cross(*this, other);
    }
}

#endif /* IDYNTREE_MOTION_VECTOR_3_H */