/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/AngularMotionVector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace iDynTree
{
    /**
     * AngularMotionVector3Semantics
     */

    // constructors
    AngularMotionVector3Semantics::AngularMotionVector3Semantics()
    {
    }

    AngularMotionVector3Semantics::AngularMotionVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    GeomVector3Semantics<AngularMotionVector3Semantics>(_body, _refBody, _coordinateFrame)
    {
    }

    AngularMotionVector3Semantics::AngularMotionVector3Semantics(const AngularMotionVector3Semantics & other):
    GeomVector3Semantics<AngularMotionVector3Semantics>(other)
    {
    }

    /**
     * AngularMotionVector3
     */

    // constructors
    AngularMotionVector3::AngularMotionVector3(const double x, const double y, const double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }

    AngularMotionVector3::AngularMotionVector3(const double* in_data, const unsigned int in_size):
    MotionVector3<AngularMotionVector3>(in_data, in_size)
    {
    }

    AngularMotionVector3::AngularMotionVector3(const AngularMotionVector3& other):
    MotionVector3<AngularMotionVector3>(other)
    {
    }

    AngularMotionVector3::AngularMotionVector3(const Vector3& other):
    MotionVector3<AngularMotionVector3>(other.data(),other.size())
    {
    }

    Rotation AngularMotionVector3::exp() const
    {
        Rotation ret;
        Eigen::Map<const Eigen::Vector3d> thisVec(this->data());
        Eigen::AngleAxisd aa(thisVec.norm(),thisVec.normalized());

        Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> >(ret.data()) = aa.toRotationMatrix();

        return ret;
    }

}
