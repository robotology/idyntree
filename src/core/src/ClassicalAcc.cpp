/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/RotationRaw.h>

#include <Eigen/Dense>

namespace iDynTree
{

// \todo TODO avoid typedef duplication
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

ClassicalAcc::ClassicalAcc(const double* in_data,
             const unsigned int in_size):
             Vector6(in_data, in_size)
{

}

ClassicalAcc::ClassicalAcc(const ClassicalAcc& other):
              Vector6(other.data(),6)
{

}


const ClassicalAcc& ClassicalAcc::changeCoordFrame(const RotationRaw& newCoordFrame)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());

    thisData.segment<3>(0) = rotData*thisData.segment<3>(0);
    thisData.segment<3>(3) = rotData*thisData.segment<3>(3);

    return *this;
}

ClassicalAcc ClassicalAcc::Zero()
{
    return ClassicalAcc();
}

Vector3 ClassicalAcc::getLinearVec3() const
{
    Vector3 ret;

    ret(0) = this->operator()(0);
    ret(1) = this->operator()(1);
    ret(2) = this->operator()(2);

    return ret;
}

Vector3 ClassicalAcc::getAngularVec3() const
{
    Vector3 ret;

    ret(0) = this->operator()(3);
    ret(1) = this->operator()(4);
    ret(2) = this->operator()(5);

    return ret;
}


void ClassicalAcc::fromSpatial(const SpatialAcc& spatialAcc,
                               const Twist& vel)
{
    // See equation 2.48 in Featherstone 2008 RNEA
    Eigen::Map<const Eigen::Vector3d> linSpatialAcc(spatialAcc.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angSpatialAcc(spatialAcc.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linTwist(vel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angTwist(vel.getAngularVec3().data());

    Eigen::Map<Eigen::Vector3d> linClassicalAcc(this->data());
    Eigen::Map<Eigen::Vector3d> angClassicalAcc(this->data()+3);

    // Linear part need to be converted
    linClassicalAcc = linSpatialAcc + angTwist.cross(linTwist);

    // Angular acceleration can be copied
    angClassicalAcc = angSpatialAcc;
}

void ClassicalAcc::toSpatial(SpatialAcc& spatialAcc, const Twist& vel) const
{
    // See equation 2.48 in Featherstone 2008 RNEA
    Eigen::Map<Eigen::Vector3d> linSpatialAcc(spatialAcc.getLinearVec3().data());
    Eigen::Map<Eigen::Vector3d> angSpatialAcc(spatialAcc.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linTwist(vel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angTwist(vel.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linClassicalAcc(this->data());
    Eigen::Map<const Eigen::Vector3d> angClassicalAcc(this->data()+3);

    // Linear part need to be converted
    linSpatialAcc = linClassicalAcc - angTwist.cross(linTwist);

    // Angular acceleration can be copied
    angSpatialAcc = angClassicalAcc;
}



}
