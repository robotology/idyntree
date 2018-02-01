/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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






