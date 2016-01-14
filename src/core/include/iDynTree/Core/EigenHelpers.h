/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_EIGEN_HELPERS_H
#define IDYNTREE_EIGEN_HELPERS_H

#include <Eigen/Dense>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>

namespace iDynTree
{

// Dynamics size toEigen methods
inline Eigen::Map<Eigen::VectorXd> toEigen(VectorDynSize & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(MatrixDynSize & mat)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

inline Eigen::Map<const Eigen::VectorXd> toEigen(const VectorDynSize & vec)
{
    return Eigen::Map<const Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const MatrixDynSize & mat)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

// Fixed size toEigen methods
template<unsigned int VecSize>
inline Eigen::Map<Eigen::Matrix<double,VecSize,1> > toEigen(VectorFixSize<VecSize> & vec)
{
    return Eigen::Map<Eigen::Matrix<double,VecSize,1> >(vec.data(),vec.size());
}

template<unsigned int VecSize>
inline Eigen::Map<const Eigen::Matrix<double,VecSize,1> > toEigen(const VectorFixSize<VecSize> & vec)
{
    return Eigen::Map<const Eigen::Matrix<double,VecSize,1> >(vec.data());
}

template<unsigned int nRows, unsigned int nCols>
inline Eigen::Map< Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> > toEigen(MatrixFixSize<nRows,nCols> & mat)
{
    return Eigen::Map< Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> >(mat.data());
}

template<unsigned int nRows, unsigned int nCols>
inline Eigen::Map< const Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> > toEigen(const MatrixFixSize<nRows,nCols> & mat)
{
    return Eigen::Map< const Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> >(mat.data());
}

// Spatia vectors
inline Eigen::Matrix<double,6,1> toEigen(const SpatialMotionVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}

inline Eigen::Matrix<double,6,1> toEigen(const SpatialForceVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor> skew(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor>() << 0.0, -vec[2], vec[1],
                                                                              vec[2], 0.0, -vec[0],
                                                                             -vec[1], vec[0], 0.0).finished();
}

}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */
