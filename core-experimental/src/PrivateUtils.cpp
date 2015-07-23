/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/PrivateUtils.h>

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

}