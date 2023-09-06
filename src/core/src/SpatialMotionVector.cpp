// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/SpatialForceVector.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Utils.h>
#include <iDynTree/PrivateUtils.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/PrivateUtils.h>

#include <Eigen/Dense>

#include <sstream>


namespace iDynTree
{

SpatialMotionVector::SpatialMotionVector(const LinearMotionVector3 & _linearVec3,
                                         const AngularMotionVector3 & _angularVec3):
                                         SpatialVector<SpatialMotionVector>(_linearVec3, _angularVec3)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialMotionVector& other):
                                         SpatialVector<SpatialMotionVector>(other)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialVector<SpatialMotionVector>& other):
                                         SpatialVector<SpatialMotionVector>(other)
{
}

SpatialMotionVector SpatialMotionVector::cross(const SpatialMotionVector& other) const
{
    SpatialMotionVector res;

    res.getLinearVec3()  = this->angularVec3.cross(other.getLinearVec3()) + this->linearVec3.cross(other.getAngularVec3());
    res.getAngularVec3() =                                                  this->angularVec3.cross(other.getAngularVec3());

    return res;
}

Matrix6x6 SpatialMotionVector::asCrossProductMatrix() const
{
    Matrix6x6 res;

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(res.data());

    retEigen.block<3,3>(0,0) = skew(toEigen(this->angularVec3));
    retEigen.block<3,3>(0,3) = skew(toEigen(this->linearVec3));
    retEigen.block<3,3>(3,0).setZero();
    retEigen.block<3,3>(3,3) = skew(toEigen(this->angularVec3));

    return res;
}


SpatialForceVector SpatialMotionVector::cross(const SpatialForceVector& other) const
{
    SpatialForceVector res;

    res.getLinearVec3()  = this->angularVec3.cross(other.getLinearVec3());
    res.getAngularVec3() = this->linearVec3.cross(other.getLinearVec3()) + this->angularVec3.cross(other.getAngularVec3());

    return res;
}

Matrix6x6 SpatialMotionVector::asCrossProductMatrixWrench() const
{
    Matrix6x6 res;

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(res.data());

    retEigen.block<3,3>(0,0) = skew(toEigen(this->angularVec3));
    retEigen.block<3,3>(0,3).setZero();
    retEigen.block<3,3>(3,0) = skew(toEigen(this->linearVec3));
    retEigen.block<3,3>(3,3) = skew(toEigen(this->angularVec3));

    return res;
}

Transform SpatialMotionVector::exp() const
{
    Transform res;

    // the linear part is affected by the left Jacobian of SO(3)
    Position newPos;
    auto J_SO3 = Rotation::leftJacobian(this->getAngularVec3());
    toEigen(newPos) = toEigen(J_SO3)*toEigen(this->getLinearVec3());
    res.setPosition(newPos);

    // the angular part instead mapped by so(3) -> SO(3) exp map
    res.setRotation(this->getAngularVec3().exp());

    return res;
}

SpatialMotionVector SpatialMotionVector::operator*(const double scalar) const
{
    SpatialMotionVector scaledVec;

    toEigen(scaledVec.linearVec3)  = scalar*toEigen(this->linearVec3);
    toEigen(scaledVec.angularVec3) = scalar*toEigen(this->angularVec3);

    return scaledVec;
}




}
