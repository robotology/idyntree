// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/PrivateUtils.h>

namespace iDynTree
{

Eigen::Matrix3d squareCrossProductMatrix(const Eigen::Vector3d & v)
{
    Eigen::Matrix3d ret;

    double vSqr[3];
    vSqr[0] = v[0]*v[0];
    vSqr[1] = v[1]*v[1];
    vSqr[2] = v[2]*v[2];

    ret <<  -(vSqr[1]+vSqr[2]),         v[0]*v[1],          v[0]*v[2],
                    v[0]*v[1], -(vSqr[0]+vSqr[2]),          v[1]*v[2],
                    v[0]*v[2],          v[1]*v[2], -(vSqr[0]+vSqr[1]);

    return ret;
}

Eigen::Matrix3d skew(const Eigen::Vector3d & vec)
{
    Eigen::Matrix3d ret;
    ret << 0.0, -vec[2], vec[1],
          vec[2], 0.0, -vec[0],
          -vec[1], vec[0], 0.0;
    return ret;
}


}
