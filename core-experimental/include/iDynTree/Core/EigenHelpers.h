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
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>

namespace iDynTree
{

/**
 * TODO \todo add templated toEigen(MatrixFixSize) to exploit compile time optimizations
 */

inline Eigen::Map<Eigen::VectorXd> toEigen(iDynTree::IRawVector & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(iDynTree::IRawMatrix & mat)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

inline Eigen::Map<const Eigen::VectorXd> toEigen(const iDynTree::IRawVector & vec)
{
    return Eigen::Map<const Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const iDynTree::IRawMatrix & mat)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

inline Eigen::Matrix<double,6,1> toEigen(SpatialMotionVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}

inline Eigen::Matrix<double,6,1> toEigen(SpatialForceVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}



}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */