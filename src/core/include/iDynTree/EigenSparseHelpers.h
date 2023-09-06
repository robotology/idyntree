// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_EIGEN_SPARSE_HELPERS_H
#define IDYNTREE_EIGEN_SPARSE_HELPERS_H

#include <Eigen/SparseCore>
#include <iDynTree/SparseMatrix.h>

namespace iDynTree
{

//SparseMatrix helpers
inline Eigen::Map< Eigen::SparseMatrix<double, Eigen::RowMajor> > toEigen(iDynTree::SparseMatrix<iDynTree::RowMajor> & mat)
{
    return Eigen::Map<Eigen::SparseMatrix<double, Eigen::RowMajor> >(mat.rows(),
                                                                     mat.columns(),
                                                                     mat.numberOfNonZeros(),
                                                                     mat.outerIndicesBuffer(),
                                                                     mat.innerIndicesBuffer(),
                                                                     mat.valuesBuffer(),
                                                                     0); //compressed format
}

inline Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor> > toEigen(const iDynTree::SparseMatrix<iDynTree::RowMajor> & mat)
{
    return Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor> >(mat.rows(),
                                                                           mat.columns(),
                                                                           mat.numberOfNonZeros(),
                                                                           mat.outerIndicesBuffer(),
                                                                           mat.innerIndicesBuffer(),
                                                                           mat.valuesBuffer(),
                                                                           0); //compressed format
}

inline Eigen::Map< Eigen::SparseMatrix<double, Eigen::ColMajor> > toEigen(iDynTree::SparseMatrix<iDynTree::ColumnMajor> & mat)
{
    return Eigen::Map<Eigen::SparseMatrix<double, Eigen::ColMajor> >(mat.rows(),
                                                                     mat.columns(),
                                                                     mat.numberOfNonZeros(),
                                                                     mat.outerIndicesBuffer(),
                                                                     mat.innerIndicesBuffer(),
                                                                     mat.valuesBuffer(),
                                                                     0); //compressed format
}

inline Eigen::Map<const Eigen::SparseMatrix<double, Eigen::ColMajor> > toEigen(const iDynTree::SparseMatrix<iDynTree::ColumnMajor> & mat)
{
    return Eigen::Map<const Eigen::SparseMatrix<double, Eigen::ColMajor> >(mat.rows(),
                                                                           mat.columns(),
                                                                           mat.numberOfNonZeros(),
                                                                           mat.outerIndicesBuffer(),
                                                                           mat.innerIndicesBuffer(),
                                                                           mat.valuesBuffer(),
                                                                           0); //compressed format
}

}

#endif /* IDYNTREE_EIGEN_SPARSE_HELPERS_H */
