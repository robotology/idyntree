/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Core/GeomVector3.h>

#include <iDynTree/Core/Rotation.h>

#include <Eigen/Dense>

namespace iDynTree
{
    typedef Eigen::Matrix<double,3,1> Vector3d;
    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;


    /**
     * GeomVector3 Method definitions ============================================
     */

    // constructors
    GeomVector3::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}

    GeomVector3::GeomVector3(const double x, const double y, const double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }

    GeomVector3::GeomVector3(const Vector3 other)
    {
        this->m_data[0] = other(0);
        this->m_data[1] = other(1);
        this->m_data[2] = other(2);
    }

    // Geometric operations
    GeomVector3 GeomVector3::changeCoordFrame(const Rotation & newCoordFrame) const
    {
        GeomVector3 result;
        Eigen::Map<const Vector3d> thisMap(this->data());
        Eigen::Map<const Matrix3dRowMajor> rotMap(newCoordFrame.data());
        Eigen::Map<Vector3d> resultMap(result.data());

        resultMap = rotMap*thisMap;

        return result;
    }

    GeomVector3 GeomVector3::compose(const GeomVector3& op1, const GeomVector3& op2) const
    {
        GeomVector3 result;
        Eigen::Map<const Vector3d> op1Data(op1.data());
        Eigen::Map<const Vector3d> op2Data(op2.data());
        Eigen::Map<Vector3d> resultData(result.data());

        resultData = op1Data + op2Data;

        return result;
    }

    GeomVector3 GeomVector3::inverse(const GeomVector3& op) const
    {
        GeomVector3 result;

        Eigen::Map<const Vector3d> opData(op.data());
        Eigen::Map<Vector3d> resultData(result.data());

        resultData = -opData;

        return result;
    }

    // dot product
    double GeomVector3::dot(const GeomVector3& other) const
    {
        Eigen::Map<const Vector3d> otherData(other.data());
        Eigen::Map<const Vector3d> thisData(this->data());

        return thisData.dot(otherData);
    }

    // overloaded operators
    GeomVector3 GeomVector3::operator+(const GeomVector3&other) const
    {
        return compose(*this, other);
    }

    GeomVector3 GeomVector3::operator-(const GeomVector3&other) const
    {
        return compose(*this, inverse(other));
    }

    GeomVector3 GeomVector3::operator-() const
    {
        return inverse(*this);
    }

    Rotation GeomVector3::exp() const
    {
        Rotation ret;
        Eigen::Map<const Eigen::Vector3d> thisVec(this->data());
        Eigen::AngleAxisd aa(thisVec.norm(), thisVec.normalized());

        Eigen::Map< Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(ret.data()) = aa.toRotationMatrix();

        return ret;
    }

    GeomVector3 GeomVector3::cross(const GeomVector3& other) const {
        GeomVector3 result;
        Eigen::Map<const Eigen::Vector3d> thisData(this->data());
        Eigen::Map<const Eigen::Vector3d> otherData(other.data());
        Eigen::Map<Eigen::Vector3d> resultData(result.data());

        resultData = thisData.cross(otherData);

        return result;
    }


}
