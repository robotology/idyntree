/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_EIGEN_HELPERS_H
#define IDYNTREE_EIGEN_HELPERS_H

#include <Eigen/Dense>
#include <iDynTree/Core/IMatrix.h>
#include <iDynTree/Core/IVector.h>

namespace iDynTree
{

inline Eigen::Map<Eigen::MatrixXd> toEigen(iDynTree::IRawVector & vec)
{
    return Eigen::Map<Eigen::MatrixXd>(vec.data(),vec.size(),1);
}

inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(iDynTree::IRawMatrix & mat)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

inline Eigen::Map<const Eigen::MatrixXd> toEigen(const iDynTree::IRawVector & vec)
{
    return Eigen::Map<const Eigen::MatrixXd>(vec.data(),vec.size(),1);
}

inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const iDynTree::IRawMatrix & mat)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}
}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */