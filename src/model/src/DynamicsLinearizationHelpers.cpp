/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/DynamicsLinearizationHelpers.h>

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree
{

SpatialForceWrtMotionDerivative SpatialForceWrtMotionDerivative::operator*(const Transform& a_X_b)
{
    SpatialForceWrtMotionDerivative ret;

    toEigen(ret) = toEigen(*this)*toEigen(a_X_b.asAdjointTransform());

    return ret;
}

SpatialMotionWrtMotionDerivative SpatialMotionWrtMotionDerivative::operator*(const Transform& a_X_b)
{
    SpatialMotionWrtMotionDerivative ret;

    toEigen(ret) = toEigen(*this)*toEigen(a_X_b.asAdjointTransform());

    return ret;
}

SpatialMotionWrtMotionDerivative operator*(const Transform & a_X_b, const SpatialMotionWrtMotionDerivative & op2)
{
    SpatialMotionWrtMotionDerivative ret;

    toEigen(ret) = toEigen(a_X_b.asAdjointTransform())*toEigen(op2);

    return ret;
}

SpatialForceWrtMotionDerivative operator*(const Transform & a_X_b, const SpatialForceWrtMotionDerivative & op2)
{
    SpatialForceWrtMotionDerivative ret;

    toEigen(ret) = toEigen(a_X_b.asAdjointTransformWrench())*toEigen(op2);

    return ret;
}


}






