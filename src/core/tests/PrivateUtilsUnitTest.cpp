/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/PrivateUtils.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/TestUtils.h>
#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

int main()
{
    double vecData[3] = {10,0,0};
    Vector3 vec(vecData,3);
    Matrix3x3 skewSquareCheck;
    Matrix3x3 skewSquare;

    toEigen(skewSquare) = squareCrossProductMatrix(toEigen(vec));

    toEigen(skewSquareCheck) =  skew(toEigen(vec))*skew(toEigen(vec));

    std::cout << skewSquare.toString() << std::endl;
    std::cout << skewSquareCheck.toString() << std::endl;

    ASSERT_EQUAL_MATRIX(skewSquare,skewSquareCheck);

    return EXIT_SUCCESS;
}