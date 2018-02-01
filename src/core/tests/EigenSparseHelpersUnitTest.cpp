/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <cstdlib>
#include <iostream>


using namespace iDynTree;

// Coeffs is not available in Eigen3.3-beta2
// See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1271
// Remove when we do need to support Eigen 3.3-beta2 (i.e. Ubuntu 16.04)
// anymore
template<typename ScalarType, typename SparseMatrixType>
const Eigen::Map<const Eigen::Array<ScalarType, Eigen::Dynamic, 1> > coeffs(const SparseMatrixType& mat)
{
    return Eigen::Array<ScalarType, Eigen::Dynamic, 1>::Map(mat.valuePtr(), mat.nonZeros());
}
template<typename ScalarType, typename SparseMatrixType>
Eigen::Map<Eigen::Array<ScalarType, Eigen::Dynamic, 1> > coeffs(SparseMatrixType& mat)
{
    return Eigen::Array<ScalarType, Eigen::Dynamic, 1>::Map(mat.valuePtr(), mat.nonZeros());
}


template <iDynTree::MatrixStorageOrdering iDynTreeOrdering, int storageOptions>
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

    SparseMatrix<iDynTreeOrdering> matrix(5, 5);
    Eigen::SparseMatrix<double, storageOptions> eig(5, 5);
    for (Triplets::const_iterator it(triplets.begin()); it != triplets.end(); ++it) {
        matrix(it->row, it->column) = it->value;
        eig.coeffRef(it->row, it->column) = it->value;
    }

    eig.makeCompressed();
    
    Eigen::Map<Eigen::SparseMatrix<double, storageOptions> > mapped = toEigen(matrix);

    ASSERT_IS_TRUE(mapped.rows() == eig.rows());
    ASSERT_IS_TRUE(mapped.cols() == eig.cols());
    ASSERT_IS_TRUE(mapped.nonZeros() == eig.nonZeros());

    auto mappedCoefficients = coeffs<double, Eigen::Map<Eigen::SparseMatrix<double, storageOptions>>>(mapped);
    auto coefficients = coeffs<double, Eigen::SparseMatrix<double, storageOptions> >(eig);

    for (unsigned i = 0; i < mappedCoefficients.size(); ++i) {
        ASSERT_EQUAL_DOUBLE(mappedCoefficients.coeff(i), coefficients.coeff(i));
    }

}

int main()
{
    sparseMatrixTest<iDynTree::RowMajor, Eigen::RowMajor>();
    sparseMatrixTest<iDynTree::ColumnMajor, Eigen::ColMajor>();

    return 0;
}

