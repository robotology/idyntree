/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/MotionVector3.h>
#include <iDynTree/Core/LinearMotionVector3.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/LinearForceVector3.h>
#include <iDynTree/Core/AngularForceVector3.h>

#include <Eigen/Dense>

#define MOTIONVECTOR3_INSTANCE_HDR \
MotionVector3<MotionT>

namespace iDynTree
{
    typedef Eigen::Matrix<double,3,1> Vector3d;

    /**
     * MotionVector3 Method definitions
     */

    // Helper template struct with embedded function for computing the cross product
    MOTIONVECTOR3_TEMPLATE_HDR
    template <class DerivedT, class OperandT>
    DerivedT MOTIONVECTOR3_INSTANCE_HDR::rawOperator<DerivedT, OperandT>::cross(const MotionVector3<MotionT>& _this, const OperandT& other)
    {
        DerivedT result;
        Eigen::Map<const Eigen::Vector3d> thisData(_this.data());
        Eigen::Map<const Eigen::Vector3d> otherData(other.data());
        Eigen::Map<Eigen::Vector3d> resultData(result.data());

        resultData = thisData.cross(otherData);

        return result;
    }

    // constructors
    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::MotionVector3(const double* in_data, const unsigned int in_size):
    GeomVector3<MotionT>(in_data, in_size)
    {}

    MOTIONVECTOR3_TEMPLATE_HDR
    MOTIONVECTOR3_INSTANCE_HDR::MotionVector3(const MotionVector3 & other):
    GeomVector3<MotionT>(other)
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


    /**
     * Instantiations for avoiding linker issues
     */

    template class MotionVector3<LinearMotionVector3>;
    template class MotionVector3<AngularMotionVector3>;

}
