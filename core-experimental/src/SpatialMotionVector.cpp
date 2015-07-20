/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialMotionVector.h"
#include "SpatialForceVector.h"
#include "PositionRaw.h"
#include "RotationRaw.h"
#include "Utils.h"
#include <sstream>
#include <Eigen/Dense>

namespace iDynTree
{

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

SpatialMotionVector::SpatialMotionVector()
{
    // Vector6() will be implicitly called
}


SpatialMotionVector::SpatialMotionVector(const double* in_data,
                                           const unsigned int in_size):
                                           Vector6(in_data, in_size)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialMotionVector& other):
                                               Vector6(other.data(),6)
{

}


SpatialMotionVector::~SpatialMotionVector()
{

}

const SpatialMotionVector& SpatialMotionVector::changePoint(const PositionRaw& newPoint)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Eigen::Vector3d> pointData(newPoint.data());

    thisData.segment<3>(0) = thisData.segment<3>(0) + pointData.cross(thisData.segment<3>(3));

    return *this;
}

const SpatialMotionVector& SpatialMotionVector::changeCoordFrame(const RotationRaw& newCoordFrame)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());

    thisData.segment<3>(0) = rotData*thisData.segment<3>(0);
    thisData.segment<3>(3) = rotData*thisData.segment<3>(3);

    return *this;
}

SpatialMotionVector SpatialMotionVector::compose(const SpatialMotionVector& op1, const SpatialMotionVector& op2)
{
    SpatialMotionVector result;

    Eigen::Map<const Vector6d> op1Data(op1.data());
    Eigen::Map<const Vector6d> op2Data(op2.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = op1Data + op2Data;

    return result;
}

SpatialMotionVector SpatialMotionVector::inverse(const SpatialMotionVector& op)
{
    SpatialMotionVector result;

    Eigen::Map<const Vector6d> opData(op.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = -opData;

    return result;
}

double SpatialMotionVector::dot(const SpatialForceVector& other) const
{
    Eigen::Map<const Vector6d> otherData(other.data());
    Eigen::Map<const Vector6d> thisData(this->data());

    return thisData.dot(otherData);
}

SpatialMotionVector SpatialMotionVector::cross(const SpatialMotionVector& other) const
{
    SpatialMotionVector res;
    Eigen::Map<const Eigen::Vector3d> op1Linear(this->data());
    Eigen::Map<const Eigen::Vector3d> op1Angular(this->data()+3);
    Eigen::Map<const Eigen::Vector3d> op2Linear(other.data());
    Eigen::Map<const Eigen::Vector3d> op2Angular(other.data()+3);
    Eigen::Map<Eigen::Vector3d> resLinear(res.data());
    Eigen::Map<Eigen::Vector3d> resAngular(res.data()+3);

    resLinear   =  op1Angular.cross(op2Linear) +  op1Linear.cross(op2Angular);
    resAngular =                                  op1Angular.cross(op2Angular);

    return res;
}

SpatialForceVector SpatialMotionVector::cross(const SpatialForceVector& other) const
{
    SpatialForceVector resForce;
    Eigen::Map<const Eigen::Vector3d> op1VelLinear(this->data());
    Eigen::Map<const Eigen::Vector3d> op1VelAngular(this->data()+3);
    Eigen::Map<const Eigen::Vector3d> op2ForceLinear(other.data());
    Eigen::Map<const Eigen::Vector3d> op2ForceAngular(other.data()+3);
    Eigen::Map<Eigen::Vector3d> resForceLinear(resForce.data());
    Eigen::Map<Eigen::Vector3d> resForceAngular(resForce.data()+3);

    resForceLinear  = op1VelAngular.cross(op2ForceLinear);
    resForceAngular = op1VelLinear.cross(op2ForceLinear) + op1VelAngular.cross(op2ForceAngular);

    return resForce;
}

SpatialMotionVector SpatialMotionVector::Zero()
{
    return SpatialMotionVector();
}


}