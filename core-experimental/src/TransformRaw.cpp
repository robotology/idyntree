/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "TransformRaw.h"
#include "Utils.h"
#include <cstdio>
#include <sstream>

#include <Eigen/Dense>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{

TransformRaw::TransformRaw(): rot(), pos()
{

}

TransformRaw::TransformRaw(const RotationRaw& _rot, const PositionRaw& _origin): rot(_rot), pos(_origin)
{

}

TransformRaw::TransformRaw(const TransformRaw& other): rot(other.rot), pos(other.pos)
{
}

TransformRaw::~TransformRaw()
{

}

TransformRaw TransformRaw::compose(const TransformRaw& op1, const TransformRaw& op2)
{
    TransformRaw result;

    Eigen::Map<const Matrix3dRowMajor> op1Rot(op1.rot.data());
    Eigen::Map<const Matrix3dRowMajor> op2Rot(op2.rot.data());
    Eigen::Map<const Eigen::Vector3d> op1Pos(op1.pos.data());
    Eigen::Map<const Eigen::Vector3d> op2Pos(op2.pos.data());

    Eigen::Map<Matrix3dRowMajor> resRot(result.rot.data());
    Eigen::Map<Eigen::Vector3d> resPos(result.pos.data());

    resRot = op1Rot*op2Rot;
    resPos = op1Rot*op2Pos+op1Pos;

    return result;
}

TransformRaw TransformRaw::inverse2(const TransformRaw& trans)
{
    TransformRaw result;

    Eigen::Map<const Matrix3dRowMajor> transRot(trans.rot.data());
    Eigen::Map<const Eigen::Vector3d> transPos(trans.pos.data());

    Eigen::Map<Matrix3dRowMajor> resRot(result.rot.data());
    Eigen::Map<Eigen::Vector3d> resPos(result.pos.data());

    resRot = transRot.transpose();
    resPos = -resRot*transPos;

    return result;
}

PositionRaw TransformRaw::apply(const TransformRaw& op1, const PositionRaw& op2)
{
    PositionRaw result;

    Eigen::Map<const Matrix3dRowMajor> op1Rot(op1.rot.data());
    Eigen::Map<const Eigen::Vector3d> op1Pos(op1.pos.data());
    Eigen::Map<const Eigen::Vector3d> op2Pos(op2.data());

    Eigen::Map<Eigen::Vector3d> resPos(result.data());

    resPos = op1Rot*op2Pos+op1Pos;

    return result;
}

TransformRaw TransformRaw::operator*(const TransformRaw& other) const
{
    return compose(*this,other);
}

TransformRaw TransformRaw::inverse() const
{
    return TransformRaw::inverse2(*this);
}

PositionRaw TransformRaw::operator*(const PositionRaw& op2) const
{
    return TransformRaw::apply(*this,op2);
}

std::string TransformRaw::toString() const
{
    std::stringstream ss;

    ss << rot.toString() << " "
       << pos.toString() << std::endl;

    return ss.str();
}

}