// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/ArticulatedBodyInertia.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Wrench.h>

#include <iDynTree/EigenHelpers.h>

#include <Eigen/Dense>

#include <iostream>
#include <sstream>

#include <cassert>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;
typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6dRowMajor;

// \todo TODO have a unique mySkew
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> mySkewABI(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}


namespace iDynTree
{

ArticulatedBodyInertia::ArticulatedBodyInertia()
{
}

ArticulatedBodyInertia::ArticulatedBodyInertia(const ArticulatedBodyInertia& other):
    linearLinear(other.linearLinear), linearAngular(other.linearAngular), angularAngular(other.angularAngular)
{

}

ArticulatedBodyInertia::ArticulatedBodyInertia(const SpatialInertia& rigidBodyInertia)
{
    Matrix6x6 rbi = rigidBodyInertia.asMatrix();
    Eigen::Map<Matrix6dRowMajor> rbiEigen(rbi.data());

    toEigen(linearLinear)  = rbiEigen.block<3,3>(0,0);
    toEigen(linearAngular) = rbiEigen.block<3,3>(0,3);
    toEigen(angularAngular) = rbiEigen.block<3,3>(3,3);
}

ArticulatedBodyInertia::ArticulatedBodyInertia(const double* in_data,
                                               const unsigned int /*in_rows*/,
                                               const unsigned int /*in_cols*/)
{
    Eigen::Map<const Matrix6dRowMajor> rbiEigen(in_data);

    toEigen(linearLinear)  = rbiEigen.block<3,3>(0,0);
    toEigen(linearAngular) = rbiEigen.block<3,3>(0,3);
    toEigen(angularAngular) = rbiEigen.block<3,3>(3,3);
}

Matrix6x6 ArticulatedBodyInertia::asMatrix() const
{
    Matrix6x6 retMat;

    Eigen::Map<Matrix6dRowMajor> retMatEigen(retMat.data());

    retMatEigen.block<3,3>(0,0) = toEigen(linearLinear);
    retMatEigen.block<3,3>(0,3) = toEigen(linearAngular);
    retMatEigen.block<3,3>(3,0) = toEigen(linearAngular).transpose();
    retMatEigen.block<3,3>(3,3) = toEigen(angularAngular);

    return retMat;
}

ArticulatedBodyInertia ArticulatedBodyInertia::combine(const ArticulatedBodyInertia& op1,
                                                       const ArticulatedBodyInertia& op2)
{
    ArticulatedBodyInertia retABI;

    toEigen(retABI.linearLinear) = toEigen(op1.linearLinear) + toEigen(op2.linearLinear);
    toEigen(retABI.linearAngular) = toEigen(op1.linearAngular) + toEigen(op2.linearAngular);
    toEigen(retABI.angularAngular) = toEigen(op1.angularAngular) + toEigen(op2.angularAngular);

    return retABI;
}

SpatialAcc ArticulatedBodyInertia::applyInverse(const Wrench& wrench) const
{
    SpatialAcc acc;

    Eigen::Matrix<double,6,1> wrenchEigen = toEigen(wrench.asVector());
    Eigen::Matrix<double,6,6> abi;

    abi.block<3,3>(0,0) = toEigen(linearLinear);
    abi.block<3,3>(0,3) = toEigen(linearAngular);
    abi.block<3,3>(3,0) = toEigen(linearAngular).transpose();
    abi.block<3,3>(3,3) = toEigen(angularAngular);

    Eigen::Matrix<double,6,1> accEigen = abi.householderQr().solve(wrenchEigen);

    toEigen(acc.getLinearVec3()) = accEigen.block<3,1>(0,0);
    toEigen(acc.getAngularVec3()) = accEigen.block<3,1>(3,0);

    return acc;
}

Matrix6x6 ArticulatedBodyInertia::getInverse() const
{
    Matrix6x6 ret;

    Eigen::Matrix<double,6,6> abi;

    abi.block<3,3>(0,0) = toEigen(linearLinear);
    abi.block<3,3>(0,3) = toEigen(linearAngular);
    abi.block<3,3>(3,0) = toEigen(linearAngular).transpose();
    abi.block<3,3>(3,3) = toEigen(angularAngular);

    toEigen(ret) = abi.inverse();

    return ret;
}



Matrix3x3& ArticulatedBodyInertia::getLinearLinearSubmatrix()
{
    return linearLinear;
}

const Matrix3x3& ArticulatedBodyInertia::getLinearLinearSubmatrix() const
{
    return linearLinear;
}

Matrix3x3& ArticulatedBodyInertia::getAngularAngularSubmatrix()
{
    return angularAngular;
}

const Matrix3x3& ArticulatedBodyInertia::getAngularAngularSubmatrix() const
{
    return angularAngular;
}

Matrix3x3& ArticulatedBodyInertia::getLinearAngularSubmatrix()
{
    return linearAngular;
}

const Matrix3x3& ArticulatedBodyInertia::getLinearAngularSubmatrix() const
{
    return linearAngular;
}

ArticulatedBodyInertia ArticulatedBodyInertia::operator+(const ArticulatedBodyInertia& other) const
{
    return combine(*this,other);
}

ArticulatedBodyInertia ArticulatedBodyInertia::operator-(const ArticulatedBodyInertia& other) const
{
    ArticulatedBodyInertia retABI;

    toEigen(retABI.linearLinear)   = toEigen(this->linearLinear)   - toEigen(other.linearLinear);
    toEigen(retABI.linearAngular)  = toEigen(this->linearAngular)  - toEigen(other.linearAngular);
    toEigen(retABI.angularAngular) = toEigen(this->angularAngular) - toEigen(other.angularAngular);

    return retABI;
}

ArticulatedBodyInertia& ArticulatedBodyInertia::operator+=(const ArticulatedBodyInertia& other)
{
    toEigen(this->linearLinear)    = toEigen(this->linearLinear)   + toEigen(other.linearLinear);
    toEigen(this->linearAngular)   = toEigen(this->linearAngular)  + toEigen(other.linearAngular);
    toEigen(this->angularAngular)  = toEigen(this->angularAngular) + toEigen(other.angularAngular);

    return *this;
}

Wrench ArticulatedBodyInertia::operator*(const SpatialAcc& other) const
{
    Wrench ret;

    toEigen(ret.getLinearVec3()) =   toEigen(this->linearLinear)*toEigen(other.getLinearVec3())
                                   + toEigen(this->linearAngular)*toEigen(other.getAngularVec3());

    toEigen(ret.getAngularVec3()) =   toEigen(this->linearAngular).transpose()*toEigen(other.getLinearVec3())
                                   + toEigen(this->angularAngular)*toEigen(other.getAngularVec3());
    return ret;

}

SpatialForceVector ArticulatedBodyInertia::operator*(const SpatialMotionVector& other) const
{
    SpatialForceVector ret;

    toEigen(ret.getLinearVec3()) =   toEigen(this->linearLinear)*toEigen(other.getLinearVec3())
                                   + toEigen(this->linearAngular)*toEigen(other.getAngularVec3());

    toEigen(ret.getAngularVec3()) =   toEigen(this->linearAngular).transpose()*toEigen(other.getLinearVec3())
                                   + toEigen(this->angularAngular)*toEigen(other.getAngularVec3());

    return ret;
}

void ArticulatedBodyInertia::zero()
{
    this->linearLinear.zero();
    this->linearAngular.zero();
    this->angularAngular.zero();
}


ArticulatedBodyInertia ArticulatedBodyInertia::ABADyadHelper(const SpatialForceVector& U, const double d)
{
    ArticulatedBodyInertia ret;

    double inv_d = 1.0/d;

    Eigen::Map<const Eigen::Vector3d> Ulin(U.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> Uang(U.getAngularVec3().data());

    toEigen(ret.getLinearLinearSubmatrix()) = (inv_d*Ulin)*Ulin.transpose();
    toEigen(ret.getLinearAngularSubmatrix()) = (inv_d*Ulin)*Uang.transpose();
    toEigen(ret.getAngularAngularSubmatrix()) = (inv_d*Uang)*Uang.transpose();

    return ret;
}

ArticulatedBodyInertia ArticulatedBodyInertia::ABADyadHelperLin(const SpatialForceVector& U, const double inv_d,
                                                                const SpatialForceVector& /*dU*/, const double d_inv_d)
{
    ArticulatedBodyInertia ret;

    Eigen::Map<const Eigen::Vector3d> Ulin(U.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> Uang(U.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> dUlin(U.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> dUang(U.getAngularVec3().data());

    toEigen(ret.getLinearLinearSubmatrix()) =   (d_inv_d*Ulin)*Ulin.transpose()
                                              + (inv_d*dUlin)*Ulin.transpose()
                                              + (inv_d*Ulin)*dUlin.transpose();
    toEigen(ret.getLinearAngularSubmatrix()) =  (d_inv_d*Ulin)*Uang.transpose()
                                              + (inv_d*dUlin)*Uang.transpose()
                                              + (inv_d*Ulin)*dUang.transpose();
    toEigen(ret.getAngularAngularSubmatrix()) = (d_inv_d*Uang)*Uang.transpose()
                                              + (inv_d*dUang)*Uang.transpose()
                                              + (inv_d*Uang)*dUang.transpose();

    return ret;
}





}
