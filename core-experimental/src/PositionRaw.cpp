/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PositionRaw.h"
#include "RotationRaw.h"
#include "SpatialMotionVector.h"
#include "SpatialForceVector.h"
#include "Utils.h"
#include <cstdio>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;
typedef Eigen::Matrix<double,6,1> Vector6d;


namespace iDynTree
{
    PositionRaw::PositionRaw()
    {
        this->zero();
    }


    PositionRaw::PositionRaw(double x, double y, double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }


    PositionRaw::PositionRaw(const PositionRaw& other)
    {
        this->m_data[0] = other.m_data[0];
        this->m_data[1] = other.m_data[1];
        this->m_data[2] = other.m_data[2];

    }

    PositionRaw::~PositionRaw()
    {

    }
    
    const PositionRaw& PositionRaw::changePoint(const PositionRaw& newPoint)
    {
        this->m_data[0] += newPoint(0);
        this->m_data[1] += newPoint(1);
        this->m_data[2] += newPoint(2);

        return *this;
    }

    const PositionRaw& PositionRaw::changeRefPoint(const PositionRaw& newRefPoint)
    {
        this->m_data[0] += newRefPoint(0);
        this->m_data[1] += newRefPoint(1);
        this->m_data[2] += newRefPoint(2);

        return *this;
    }

    PositionRaw PositionRaw::compose(const PositionRaw& op1, const PositionRaw& op2)
    {
        PositionRaw result;
        result(0) = op1(0) + op2(0);
        result(1) = op1(1) + op2(1);
        result(2) = op1(2) + op2(2);
        return result;
    }

    PositionRaw PositionRaw::inverse(const PositionRaw& op)
    {
        PositionRaw result;
        result(0) = -op.m_data[0];
        result(1) = -op.m_data[1];
        result(2) = -op.m_data[2];
        return result;
    }

    SpatialMotionVector PositionRaw::changePointOf(const SpatialMotionVector & other) const
    {
        SpatialMotionVector result;

        Eigen::Map<const Eigen::Vector3d> thisPos(this->data());
        Eigen::Map<const Vector6d> otherTwist(other.data());
        Eigen::Map<Vector6d> resTwist(result.data());

        resTwist.segment<3>(0) =  otherTwist.segment<3>(0)+thisPos.cross(otherTwist.segment<3>(3));
        resTwist.segment<3>(3) =  otherTwist.segment<3>(3);

        return result;
    }

    SpatialForceVector PositionRaw::changePointOf(const SpatialForceVector & other) const
    {
        SpatialForceVector result;

        Eigen::Map<const Eigen::Vector3d> thisPos(this->data());
        Eigen::Map<const Vector6d> otherWrench(other.data());
        Eigen::Map<Vector6d> resWrench(result.data());

        resWrench.segment<3>(0) = otherWrench.segment<3>(0);
        resWrench.segment<3>(3) = thisPos.cross(otherWrench.segment<3>(0))+otherWrench.segment<3>(3);

        return result;
    }

    std::string PositionRaw::toString() const
    {
        std::stringstream ss;

        ss << " x " << this->m_data[0]
        << " y " << this->m_data[1]
        << " z " << this->m_data[2];

        return ss.str();
    }

    std::string PositionRaw::reservedToString() const
    {
        return this->toString();
    }




}