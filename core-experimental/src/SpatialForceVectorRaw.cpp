/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialForceVectorRaw.h>
#include <iDynTree/Core/SpatialMotionVectorRaw.h>
#include <iDynTree/Core/PositionRaw.h>
#include <iDynTree/Core/RotationRaw.h>
#include <iDynTree/Core/Utils.h>

#include <Eigen/Dense>

#include <sstream>

namespace iDynTree
{

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

SpatialForceVectorRaw::SpatialForceVectorRaw()
{
    // Vector6() will be implicitly called
}


SpatialForceVectorRaw::SpatialForceVectorRaw(const double* in_data, const unsigned int in_size): Vector6(in_data, in_size)
{
}

SpatialForceVectorRaw::SpatialForceVectorRaw(const SpatialForceVectorRaw& other): Vector6(other.data(),6)
{

}

SpatialForceVectorRaw::~SpatialForceVectorRaw()
{

}

const SpatialForceVectorRaw& SpatialForceVectorRaw::changePoint(const PositionRaw& newPoint)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Eigen::Vector3d> pointData(newPoint.data());

    thisData.segment<3>(0) = thisData.segment<3>(0) + pointData.cross(thisData.segment<3>(3));

    return *this;
}

const SpatialForceVectorRaw& SpatialForceVectorRaw::changeCoordFrame(const RotationRaw& newCoordFrame)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());

    thisData.segment<3>(0) = rotData*thisData.segment<3>(0);
    thisData.segment<3>(3) = rotData*thisData.segment<3>(3);

    return *this;
}

SpatialForceVectorRaw SpatialForceVectorRaw::compose(const SpatialForceVectorRaw& op1, const SpatialForceVectorRaw& op2)
{
    SpatialForceVectorRaw result;

    Eigen::Map<const Vector6d> op1Data(op1.data());
    Eigen::Map<const Vector6d> op2Data(op2.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = op1Data + op2Data;

    return result;
}

SpatialForceVectorRaw SpatialForceVectorRaw::inverse(const SpatialForceVectorRaw& op)
{
    SpatialForceVectorRaw result;

    Eigen::Map<const Vector6d> opData(op.data());
    Eigen::Map<Vector6d> resultData(result.data());

    resultData = -opData;

    return result;
}

double SpatialForceVectorRaw::dot(const SpatialMotionVectorRaw& other) const
{
    Eigen::Map<const Vector6d> otherData(other.data());
    Eigen::Map<const Vector6d> thisData(this->data());

    return thisData.dot(otherData);
}

/*
SpatialForceVectorRaw SpatialForceVectorRaw::operator+(const SpatialForceVectorRaw& other) const
{
    return compose(*this,other);
}

SpatialForceVectorRaw SpatialForceVectorRaw::operator-() const
{
    return inverse(*this);
}

SpatialForceVectorRaw SpatialForceVectorRaw::operator-(const SpatialForceVectorRaw& other) const
{
    return compose(*this,inverse(other));
}*/


SpatialForceVectorRaw SpatialForceVectorRaw::Zero()
{
    return SpatialForceVectorRaw();
}


}