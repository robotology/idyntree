// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/MatrixView.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/EigenHelpers.h>

using namespace iDynTree;

template <class T, class U>
void areMatricesEqual(const T & mat1, const U & mat2)
{
    ASSERT_EQUAL_DOUBLE(mat1.rows(), mat2.rows());
    ASSERT_EQUAL_DOUBLE(mat1.cols(), mat2.cols());

    ASSERT_EQUAL_MATRIX(mat1, mat2);
}

int main()
{
    // test with iDynTree matrix
    MatrixDynSize mat1(27, 41);
    getRandomMatrix(mat1);
    MatrixDynSize subMat1(10, 23);
    toEigen(subMat1) = toEigen(mat1).block(3, 5, 10, 23);

    MatrixView<double> view1(mat1);
    areMatricesEqual(view1, mat1);

    auto subView1 = view1.block(3, 5, 10, 23);
    areMatricesEqual(subView1, subMat1);

    // Test with eigen column major matrix
    Eigen::MatrixXd mat2(27, 41);
    mat2.setRandom();
    Eigen::MatrixXd subMat2;
    subMat2 = mat2.block(3, 5, 10, 23);

    MatrixView<double> view2(mat2);
    areMatricesEqual(view2, mat2);

    auto subView2 = view2.block(3, 5, 10, 23);
    areMatricesEqual(subView2, subMat2);

    // Test with eigen row major matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat3(27, 41);
    mat3.setRandom();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> subMat3;
    subMat3 = mat3.block(3, 5, 10, 23);

    MatrixView<double> view3(mat3);
    areMatricesEqual(view3, mat3);

    auto view4 = make_matrix_view(mat3);
    areMatricesEqual(view4, mat3);

    auto subView3 = view3.block(3, 5, 10, 23);
    areMatricesEqual(subView3, subMat3);

    return 0;
}
