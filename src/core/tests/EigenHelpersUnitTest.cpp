/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <cstdlib>

using namespace iDynTree;

void validateSkewUnskew(const Vector3 randVec)
{
    Vector3 randVecCheck;

    toEigen(randVecCheck) = unskew(skew(toEigen(randVec)));

    ASSERT_EQUAL_VECTOR(randVec,randVecCheck);
}

void sparseMatrixTest()
{
    SparseMatrix matrix(5,5);
    matrix(2,0) = 7;
    matrix(0,1) = 3;
    matrix(1,0) = 22;
    matrix(1,4) = 17;
    matrix(2,1) = 5;
    matrix(2,3) = 1;
    matrix(4,2) = 14;
    matrix(4,4) = 8;

    Eigen::SparseMatrix<double, Eigen::RowMajor> eig(5, 5);
    eig.coeffRef(0,1) = 3;
    eig.coeffRef(1,0) = 22;
    eig.coeffRef(1,4) = 17;
    eig.coeffRef(2,0) = 7;
    eig.coeffRef(2,1) = 5;
    eig.coeffRef(2,3) = 1;
    eig.coeffRef(4,2) = 14;
    eig.coeffRef(4,4) = 8;

    eig.makeCompressed();

    std::cout << "Matrix:\n" << matrix.description(true) << "\n";
    std::cout << "Eigen Map:\n" << toEigen(matrix) << "\n";

    std::cout << "Eigen:\n" << eig << "\n";
}

int main()
{
    Vector3 vec;

    vec(0) = 1;
    vec(1) = 2;
    vec(2) = 3;

    validateSkewUnskew(vec);

    sparseMatrixTest();

    return EXIT_SUCCESS;
}
