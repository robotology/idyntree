/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <cstdlib>

using namespace iDynTree;

void validateSkewUnskew(const Vector3 randVec)
{
    Vector3 randVecCheck;

    toEigen(randVecCheck) = unskew(skew(toEigen(randVec)));

    ASSERT_EQUAL_VECTOR(randVec,randVecCheck);
}

void testSpatialVectors()
{
    SpatialForceVector zeroVector;
    zeroVector.zero();

    for (size_t i = 0; i < zeroVector.size(); ++i) {
        ASSERT_EQUAL_DOUBLE(zeroVector(i), 0);
    }
}

void testSpanToEigen(const Vector3& input) {
    Vector3 randVec, check;
    getRandomVector(randVec);
    Span<double> spanCheck = make_span(check);
    toEigen(spanCheck) = toEigen(randVec);
    ASSERT_EQUAL_VECTOR(randVec, check);
    toEigen(make_span(check)) = toEigen(randVec);
    toEigen(randVec) = toEigen(make_span(check));
    toEigen(check) = toEigen(make_span(input));
    ASSERT_EQUAL_VECTOR(input, check);
}

template <class MatrixType>
void testMatrixToEigen(const MatrixType& input) {
    MatrixDynSize randMat(input.rows(), input.cols()), check(input.rows(), input.cols());
    getRandomMatrix(randMat);
    MatrixView<double> matrixViewCheck(check);
    toEigen(matrixViewCheck) = toEigen(randMat);
    ASSERT_EQUAL_MATRIX(randMat, check);
    toEigen(make_matrix_view(check)) = toEigen(randMat);
    toEigen(randMat) = toEigen(make_matrix_view(check));
    toEigen(check) = toEigen(make_matrix_view(input));
    ASSERT_EQUAL_MATRIX(input, check);
}

void checkMatrixViewStorageOrder(const Eigen::MatrixXd& input) {
    auto matrixView = make_matrix_view(input);

    // The toEigen returns a RowMajor matrix even if the original matrix is ColMajor.
    // The Eigen::Stride is chosen to have the coherent behaviour.
    ASSERT_EQUAL_MATRIX(input, toEigen(matrixView));
}

int main()
{
    Vector3 vec;

    vec(0) = 1;
    vec(1) = 2;
    vec(2) = 3;

    validateSkewUnskew(vec);

    testSpatialVectors();

    testSpanToEigen(vec);


    // test the matrix view
    Matrix2x3 mat1;
    getRandomMatrix(mat1);
    testMatrixToEigen(mat1);

    MatrixDynSize mat2(12,31);
    getRandomMatrix(mat2);
    testMatrixToEigen(mat2);

    Eigen::MatrixXd mat3(12,31);
    getRandomMatrix(mat3);
    testMatrixToEigen(mat3);

    checkMatrixViewStorageOrder(mat3);

    return EXIT_SUCCESS;
}
