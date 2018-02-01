/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_EIGEN_SPARSE_HELPERS_H
#define IDYNTREE_EIGEN_SPARSE_HELPERS_H

#include <Eigen/SparseCore>
#include <iDynTree/Core/SparseMatrix.h>

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
