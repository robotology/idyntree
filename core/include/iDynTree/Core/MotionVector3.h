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
template <class MotionT>

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
    class MotionVector3: public GeomVector3<MotionT>
    {
    private:
        /**
         * Helper template struct with embedded function for computing the cross product
         */
        template <class DerivedT, class OperandT>
        struct rawOperator
        {
            static DerivedT cross(const MotionVector3<MotionT>& _this, const OperandT& other);
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
        typedef typename MotionDerivativeOf<MotionT, LinearMotionVector3>::Type MotionCrossLinM;
        typedef typename MotionDerivativeOf<MotionT, AngularMotionVector3>::Type MotionCrossAngM;
        typedef typename MotionDerivativeOf<MotionT, LinearForceVector3>::Type MotionCrossLinF;
        typedef typename MotionDerivativeOf<MotionT, AngularForceVector3>::Type MotionCrossAngF;
        
        /* Cross products */
        MotionCrossLinM cross(const LinearMotionVector3& other) const;
        MotionCrossAngM cross(const AngularMotionVector3& other) const;
        MotionCrossLinF cross(const LinearForceVector3& other) const;
        MotionCrossAngF cross(const AngularForceVector3& other) const;
    };

}

#endif /* IDYNTREE_MOTION_VECTOR_3_H */