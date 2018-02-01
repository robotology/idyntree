/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/LinearForceVector3.h>

namespace iDynTree
{
    /**
     * LinearForceVector3Semantics
     */

    // constructors
    LinearForceVector3Semantics::LinearForceVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    ForceVector3Semantics<LinearForceVector3Semantics>(_body, _refBody, _coordinateFrame)
    {
    }

    LinearForceVector3Semantics::LinearForceVector3Semantics(const LinearForceVector3Semantics & other):
    ForceVector3Semantics<LinearForceVector3Semantics>(other)
    {
    }


    /**
     * LinearForceVector3
     */

    // constructors
    LinearForceVector3::LinearForceVector3(const double x, const double y, const double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }


    LinearForceVector3::LinearForceVector3(const double* in_data, const unsigned int in_size):
    ForceVector3<LinearForceVector3>(in_data, in_size)
    {
    }

    LinearForceVector3::LinearForceVector3(const LinearForceVector3 & other):
    ForceVector3<LinearForceVector3>(other)
    {
    }

    LinearForceVector3::LinearForceVector3(const Vector3& other):
    ForceVector3<LinearForceVector3>(other.data(), other.size())
    {
    }


}
