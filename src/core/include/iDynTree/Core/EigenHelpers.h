/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_EIGEN_HELPERS_H
#define IDYNTREE_EIGEN_HELPERS_H

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Utils.h>

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

// Spatial vectors
inline void fromEigen(SpatialMotionVector & vec, const Eigen::Matrix<double,6,1> & eigVec)
{
    toEigen(vec.getLinearVec3()) = eigVec.segment<3>(0);
    toEigen(vec.getAngularVec3()) = eigVec.segment<3>(3);
}

inline void fromEigen(SpatialForceVector & vec, const Eigen::Matrix<double,6,1> & eigVec)
{
    toEigen(vec.getLinearVec3()) = eigVec.segment<3>(0);
    toEigen(vec.getAngularVec3()) = eigVec.segment<3>(3);
}

inline void fromEigen(Transform & trans, const Eigen::Matrix4d & eigMat)
{
    Rotation rot;
    Position pos;

    toEigen(rot) = eigMat.block<3,3>(0,0);
    toEigen(pos) = eigMat.block<3,1>(0,3);

    trans.setRotation(rot);
    trans.setPosition(pos);
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor> skew(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor>() << 0.0, -vec[2], vec[1],
                                                                              vec[2], 0.0, -vec[0],
                                                                             -vec[1], vec[0], 0.0).finished();
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> unskew(const Eigen::MatrixBase<Derived> & mat)
{
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 1>() << mat(2,1), mat(0,2), mat(1,0) ).finished();
}


/**
 * Submatrix helpers
 */
template<unsigned int nRows, unsigned int nCols, typename iDynTreeMatrixType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const MatrixFixSize<nRows,nCols>& subMat)
{
#if __cplusplus > 199711L
    static_assert(!std::is_base_of<iDynTreeMatrixType, SparseMatrix>::value,
                  "You cannot set a subMatrix on a SparseMatrix.");
#endif

    toEigen(mat).block(rowRange.offset,colRange.offset,rowRange.size,colRange.size) = toEigen(subMat);
    return;
}

template<typename iDynTreeMatrixType, typename EigMatType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const EigMatType& subMat)
{
#if __cplusplus > 199711L
    static_assert(!std::is_base_of<iDynTreeMatrixType, SparseMatrix>::value,
                  "You cannot set a subMatrix on a SparseMatrix.");
#endif
    toEigen(mat).block(rowRange.offset,colRange.offset,rowRange.size,colRange.size) = subMat;
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const double subMat)
{
#if __cplusplus > 199711L
    static_assert(!std::is_base_of<iDynTreeMatrixType, SparseMatrix>::value,
                  "You cannot set a subMatrix on a SparseMatrix.");
#endif
    assert(rowRange.size == 1);
    assert(colRange.size == 1);
    mat(rowRange.offset,colRange.offset) = subMat;
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrixToIdentity(iDynTreeMatrixType& mat,
                                   const IndexRange rowRange,
                                   const IndexRange colRange)
{
#if __cplusplus > 199711L
    static_assert(!std::is_base_of<iDynTreeMatrixType, SparseMatrix>::value,
                  "You cannot set a subMatrix on a SparseMatrix.");
#endif
    assert(rowRange.size == colRange.size);
    for(int i=0; i < rowRange.size; i++)
    {
        mat(rowRange.offset+i,colRange.offset+i) = 1.0;
    }
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrixToMinusIdentity(iDynTreeMatrixType& mat,
                                        const IndexRange rowRange,
                                        const IndexRange colRange)
{
#if __cplusplus > 199711L
    static_assert(!std::is_base_of<iDynTreeMatrixType, SparseMatrix>::value,
                  "You cannot set a subMatrix on a SparseMatrix.");
#endif
    assert(rowRange.size == colRange.size);
    for(int i=0; i < rowRange.size; i++)
    {
        mat(rowRange.offset+i,colRange.offset+i) = -1.0;
    }
    return;
}


template<unsigned int size>
inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const VectorFixSize<size>& subVec)
{
    toEigen(vec).segment(range.offset,range.size) = toEigen(subVec);
    return;
}

inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         double subVec)
{
    assert(range.size==1);
    vec(range.offset) = subVec;
    return;
}

template<typename T>
inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const T& subVec)
{
    toEigen(vec).segment(range.offset,range.size) = subVec;
    return;
}

}


//SparseMatrix helpers
inline Eigen::Map< Eigen::SparseMatrix<double, Eigen::RowMajor> > toEigen(iDynTree::SparseMatrix & mat)
{
    return Eigen::Map<Eigen::SparseMatrix<double, Eigen::RowMajor> >(mat.rows(),
                                                                     mat.columns(),
                                                                     mat.numberOfNonZeros(),
                                                                     mat.outerIndecesBuffer(),
                                                                     mat.innerIndecesBuffer(),
                                                                     mat.valuesBuffer(),
                                                                     0); //compressed format
}

inline Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor> > toEigen(const iDynTree::SparseMatrix & mat)
{
    return Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor> >(mat.rows(),
                                                                           mat.columns(),
                                                                           mat.numberOfNonZeros(),
                                                                           mat.outerIndecesBuffer(),
                                                                           mat.innerIndecesBuffer(),
                                                                           mat.valuesBuffer(),
                                                                           0); //compressed format
}


#endif /* IDYNTREE_EIGEN_HELPERS_H */
