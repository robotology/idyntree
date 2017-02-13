/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <cstdlib>
#include <iostream>


using namespace iDynTree;

void sparseMatrixTest()
{
    Triplets triplets;
    triplets.pushTriplet(iDynTree::Triplet(2, 0, 7));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 3));
    triplets.pushTriplet(iDynTree::Triplet(1, 0, 22));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, 17));
    triplets.pushTriplet(iDynTree::Triplet(2, 1, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 3, 1));
    triplets.pushTriplet(iDynTree::Triplet(4, 2, 14));
    triplets.pushTriplet(iDynTree::Triplet(4, 4, 8));

    SparseMatrix matrix(5,5);
    Eigen::SparseMatrix<double, Eigen::RowMajor> eig(5, 5);
    for (Triplets::const_iterator it(triplets.begin()); it != triplets.end(); ++it) {
        matrix(it->row, it->column) = it->value;
        eig.coeffRef(it->row, it->column) = it->value;
    }

    eig.makeCompressed();
    
    Eigen::Map<Eigen::SparseMatrix<double, Eigen::RowMajor> > mapped = toEigen(matrix);

    ASSERT_IS_TRUE(mapped.rows() == eig.rows());
    ASSERT_IS_TRUE(mapped.cols() == eig.cols());
    ASSERT_IS_TRUE(mapped.nonZeros() == eig.nonZeros());

    auto mappedCoefficients = mapped.coeffs();
    auto coefficients = eig.coeffs();

    for (unsigned i = 0; i < mappedCoefficients.size(); ++i) {
        ASSERT_EQUAL_DOUBLE(mappedCoefficients.coeff(i), coefficients.coeff(i));
    }

}

int main()
{
    sparseMatrixTest();

    return 0;
}

