/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RotationRaw.h"
#include "PositionRaw.h"
#include "Utils.h"
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{
RotationRaw::RotationRaw()
{
    Eigen::Map<Matrix3dRowMajor> thisData(this->privateData);

    thisData.setIdentity();
}

RotationRaw::RotationRaw(double xx, double xy, double xz,
                         double yx, double yy, double yz,
                         double zx, double zy, double zz)
{
    this->privateData[0] = xx;
    this->privateData[1] = xy;
    this->privateData[2] = xz;
    this->privateData[3] = yx;
    this->privateData[4] = yy;
    this->privateData[5] = yz;
    this->privateData[6] = zx;
    this->privateData[7] = zy;
    this->privateData[8] = zz;
}


RotationRaw::RotationRaw(const RotationRaw& other)
{
    Eigen::Map<Matrix3dRowMajor> thisData(this->privateData);
    Eigen::Map<const Matrix3dRowMajor> otherData(other.privateData);

    thisData = otherData;
}

RotationRaw::~RotationRaw()
{

}

double& RotationRaw::operator()(const unsigned int row, const unsigned int col)
{
    return this->privateData[3*row+col];
}

double RotationRaw::operator()(const unsigned int row, const unsigned int col) const
{
    return this->privateData[3*row+col];
}

double RotationRaw::getVal(const unsigned int row, const unsigned int col) const
{
    if( row > this->rows() ||
        col  > this->cols() )
    {
        reportError("RotationRaw","getVal","indeces out of bounds");
        return 0.0;
    }

    return this->privateData[3*row+col];
}

bool RotationRaw::setVal(const unsigned int row, const unsigned int col, const double new_el)
{
    if( row > this->rows() ||
        col   > this->cols() )
    {
        reportError("RotationRaw","setVal","indeces out of bounds");
        return false;
    }

    this->privateData[3*row+col] = new_el;
    return true;
}

unsigned int RotationRaw::rows() const
{
    return 3;
}

unsigned int RotationRaw::cols() const
{
    return 3;
}

double* RotationRaw::data()
{
    return this->privateData;
}

const double* RotationRaw::data() const
{
    return this->privateData;
}

const RotationRaw& RotationRaw::changeOrientFrame(const RotationRaw& newOrientFrame)
{
    Eigen::Map<Matrix3dRowMajor> thisData(this->privateData);
    Eigen::Map<Matrix3dRowMajor> newOrientFrameData(this->privateData);

    thisData = thisData*newOrientFrameData;

    return *this;
}

const RotationRaw& RotationRaw::changeRefOrientFrame(const RotationRaw& newRefOrientFrame)
{
    Eigen::Map<Matrix3dRowMajor> thisData(this->privateData);
    Eigen::Map<Matrix3dRowMajor> newRefOrientFrameData(this->privateData);

    thisData = newRefOrientFrameData*thisData;

    return *this;
}

RotationRaw RotationRaw::compose(const RotationRaw& op1, const RotationRaw& op2)
{
    RotationRaw result;

    Eigen::Map<const Matrix3dRowMajor> op1Data(op1.privateData);
    Eigen::Map<const Matrix3dRowMajor> op2Data(op2.privateData);
    Eigen::Map<Matrix3dRowMajor> resultData(result.privateData);

    resultData = op1Data*op2Data;

    return result;
}

RotationRaw RotationRaw::inverse2(const RotationRaw& orient)
{
    RotationRaw result;

    Eigen::Map<const Matrix3dRowMajor> orientData(orient.privateData);
    Eigen::Map<Matrix3dRowMajor> resultData(result.privateData);

    resultData = orientData.transpose();

    return result;
}

PositionRaw RotationRaw::transform(const RotationRaw& op1, const PositionRaw& op2)
{
    PositionRaw result;

    Eigen::Map<const Matrix3dRowMajor> op1Data(op1.privateData);
    Eigen::Map<const Eigen::Vector3d> op2Data(op2.data());
    Eigen::Map<Eigen::Vector3d> resultData(result.data());

    resultData = op1Data*op2Data;

    return result;
}

RotationRaw RotationRaw::inverse() const
{
    return RotationRaw::inverse2(*this);
}

RotationRaw RotationRaw::operator*(const RotationRaw& other) const
{
    return compose(*this,other);
}

PositionRaw RotationRaw::operator*(const PositionRaw& other) const
{
    return transform(*this,other);
}

RotationRaw RotationRaw::RotX(const double angle)
{
    RotationRaw result;
    Eigen::Map<Matrix3dRowMajor> thisData(result.data());
    thisData = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).matrix();

    return result;
}

RotationRaw RotationRaw::RotY(const double angle)
{
    RotationRaw result;
    Eigen::Map<Matrix3dRowMajor> thisData(result.data());
    thisData = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).matrix();

    return result;
}

RotationRaw RotationRaw::RotZ(const double angle)
{
    RotationRaw result;
    Eigen::Map<Matrix3dRowMajor> thisData(result.data());
    thisData = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();

    return result;
}

RotationRaw RotationRaw::RPY(const double roll, const double pitch, const double yaw)
{
    return RotX(roll)*RotY(pitch)*RotZ(yaw);
}

std::string RotationRaw::toString() const
{
   std::stringstream ss;

    ss << this->privateData[0]
       << " " << this->privateData[1]
       << " " << this->privateData[2] << std::endl;
    ss << this->privateData[3]
       << " " << this->privateData[4]
       << " " << this->privateData[5] << std::endl;
    ss << this->privateData[6]
       << " " << this->privateData[7]
       << " " << this->privateData[8] << std::endl;

    return ss.str();
}

std::string RotationRaw::reservedToString() const
{
    return this->toString();
}


}