/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialForceVector.h"
#include "SpatialMotionVector.h"
#include "PositionRaw.h"
#include "RotationRaw.h"
#include "Utils.h"
#include <sstream>
#include <Eigen/Dense>

namespace iDynTree
{

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

SpatialForceVector::SpatialForceVector()
{
    // Vector6() will be implicitly called
}


SpatialForceVector::SpatialForceVector(const double* in_data, const unsigned int in_size): Vector6(in_data, in_size)
{
}

SpatialForceVector::SpatialForceVector(const SpatialForceVector& other): Vector6(other.data(),6)
{

}

SpatialForceVector::~SpatialForceVector()
{

}

const SpatialForceVector& SpatialForceVector::changePoint(const PositionRaw& newPoint)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Eigen::Vector3d> pointData(newPoint.data());

    thisData.segment<3>(0) = thisData.segment<3>(0) + pointData.cross(thisData.segment<3>(3));

    return *this;
}

const SpatialForceVector& SpatialForceVector::changeCoordFrame(const RotationRaw& newCoordFrame)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());

    thisData.segment<3>(0) = rotData*thisData.segment<3>(0);
    thisData.segment<3>(3) = rotData*thisData.segment<3>(3);

    return *this;
}

SpatialForceVector SpatialForceVector::compose(const SpatialForceVector& op1, const SpatialForceVector& op2)
{
    SpatialForceVector result;

    Eigen::Map<const Vector6d> op1Data(op1.data());
    Eigen::Map<const Vector6d> op2Data(op2.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = op1Data + op2Data;

    return result;
}

SpatialForceVector SpatialForceVector::inverse(const SpatialForceVector& op)
{
    SpatialForceVector result;

    Eigen::Map<const Vector6d> opData(op.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = -opData;

    return result;
}

double SpatialForceVector::dot(const SpatialMotionVector& other) const
{
    Eigen::Map<const Vector6d> otherData(other.data());
    Eigen::Map<const Vector6d> thisData(this->data());

    return thisData.dot(otherData);
}

/*
SpatialForceVector SpatialForceVector::operator+(const SpatialForceVector& other) const
{
    return compose(*this,other);
}

SpatialForceVector SpatialForceVector::operator-() const
{
    return inverse(*this);
}

SpatialForceVector SpatialForceVector::operator-(const SpatialForceVector& other) const
{
    return compose(*this,inverse(other));
}*/


SpatialForceVector SpatialForceVector::Zero()
{
    return SpatialForceVector();
}


}