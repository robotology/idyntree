/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_YARP_CONVERSIONS_H
#define IDYNTREE_YARP_CONVERSIONS_H

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/Core/Wrench.h>

namespace iDynTree
{

class Direction;
class VectorDynSize;
class Transform;

/**
 * Convert a yarp::sig::Vector to a iDynTree::Wrench
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeWrench iDynTree::Wrench output
 * @return true if conversion was successful, false otherwise
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench);

/**
 * Convert a iDynTree::Wrench to a yarp::sig::Vector
 * @param iDynTreeWrench iDynTree::Wrench input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 * \ingroup iDynTreeYARP
 */
bool toYarp(const iDynTree::Wrench & iDynTreeWrench, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector to a iDynTree::Position
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreePosition iDynTree::Position output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Position & iDynTreePosition);

/**
 * Convert a yarp::sig::Vector to a iDynTree::Vector3
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreePosition iDynTree::Vector3 output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Vector3 & iDynTreeVector3);

/**
 * Convert a iDynTree::Position to a yarp::sig::Vector of 3 elements.
 * @param iDynTreePosition iDynTree::Position input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was sucessful, false otherwise
 * \ingroup iDynTreeYARP
 */
bool toYarp(const iDynTree::Position & iDynTreePosition, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector of 3 elements to a iDynTree::Direction
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeDirection iDynTree::Direction output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 *
 * \note the direction vector will be normalized to have unit norm.
 *
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Direction & iDynTreeDirection);

/**
 * Convert a iDynTree::Direction to a yarp::sig::Vector of 3 elements.
 * @param iDynTreeDirection iDynTree::Position input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was sucessful, false otherwise
 *
 * \ingroup iDynTreeYARP
 */
bool toYarp(const iDynTree::Vector3 & iDynTreeDirection, yarp::sig::Vector & yarpVector);

/**
 * Convert a 4x4 yarp::sig::Matrix representing an homegeneous matrix to a iDynTree::Transform
 * @param yarpHomogeneousMatrix yarp::sig::Matrix 4x4 homegeneous matrix input
 * @param iDynTreeTransform iDynTree::Transform output
 * @return true if conversion was successful, false otherwise
 *
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Matrix & yarpHomogeneousMatrix, iDynTree::Transform & iDynTreeTransform);

/**
 * Convert a iDynTree::Transform to a 4x4 yarp::sig::Matrix representing an homegeneous matrix
 * @param iDynTreeTransform iDynTree::Transform input
 * @param yarpHomogeneousMatrix yarp::sig::Matrix 4x4 homegeneous matrix output
 * @return true if conversion was successful, false otherwise
 *
 * \ingroup iDynTreeYARP
 */
bool toYarp(const iDynTree::Transform & iDynTreeTransform,  yarp::sig::Matrix & yarpHomogeneousMatrix);

/**
 * Convert a yarp::sig::Vector to a iDynTree::VectorDynSize
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeVector iDynTree::VectorDynSize output
 * @return true if conversion was successful, false otherwise
 * \note the output VectorDynSize will be resized if necessary.
 *
 * \ingroup iDynTreeYARP
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::VectorDynSize & iDynTreeVector);

/**
 * Convert a iDynTree::VectorFixSize to a yarp::sig::Vector
 * @param iDynTreeVector iDynTree::VectorFixSize input
 * @param yarpVector yarp::sig::Vector output
 * \note the output yarp::sig::Vector will be resized if necessary.
 *
 * \ingroup iDynTreeYARP
 */
template <typename VectorType>
void toYarp(const VectorType& iDynTreeVector, yarp::sig::Vector& yarpVector)
{
    size_t vecSize = iDynTreeVector.size();
    if( yarpVector.size() != vecSize )
    {
        yarpVector.resize(vecSize);
    }

    memcpy(yarpVector.data(),iDynTreeVector.data(),vecSize*sizeof(double));
}

/**
 * Convert a iDynTree::MatrixFixSize to a yarp::sig::Matrix
 * @param iDynTreeMatrix iDynTree::MatrixFixSize input
 * @param yarpMatrix yarp::sig::Matrix output
 * \note the output yarp::sig::Matrix will be resized if necessary.
 *
 * \ingroup iDynTreeYARP
 */
template <typename MatrixType>
void toYarp(const MatrixType& iDynTreeMatrix, yarp::sig::Matrix& yarpMatrix)
{
    size_t rows = iDynTreeMatrix.rows();
    size_t cols = iDynTreeMatrix.cols();

    if( static_cast<size_t>(yarpMatrix.rows()) != rows ||
        static_cast<size_t>(yarpMatrix.cols()) != cols )
    {
        yarpMatrix.resize(rows,cols);
    }

    // Here we explot the fact that both YARP and iDynTree store
    // Matrices in row major forma .
    memcpy(yarpMatrix.data(),iDynTreeMatrix.data(),rows*cols*sizeof(double));
}

/**
 * Convert a yarp::sig::Vector to a iDynTree::VectorFixSize
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeVector iDynTree::VectorFixSize output
 * @return true if conversion was successful, false otherwise
 *        (if the input yarpMatrix has size different from the output VectorFixSize)
 *
 * \ingroup iDynTreeYARP
 */
template <typename VectorType>
bool toiDynTree(const yarp::sig::Vector& yarpVector, VectorType& iDynTreeVector)
{
    size_t vecSize = iDynTreeVector.size();
    if( yarpVector.size() != vecSize )
    {
        return false;
    }

    memcpy(iDynTreeVector.data(),yarpVector.data(),vecSize*sizeof(double));
    return true;
}

/**
 * Convert a yarp::sig::Matrix to a iDynTree::MatrixFixSize
 * @param yarpMatrix yarp::sig::Matrix input
 * @param iDynTreeMatrix iDynTree::MatrixFixSize output
 * @return true if conversion was successful, false otherwise
 *        (if the input yarpMatrix has size different from the MatrixFixSize)
 *
 * \ingroup iDynTreeYARP
 */
template <typename MatrixType>
bool toiDynTree(const yarp::sig::Matrix& yarpMatrix, MatrixType& iDynTreeMatrix)
{
    size_t rows = iDynTreeMatrix.rows();
    size_t cols = iDynTreeMatrix.cols();

    if( static_cast<size_t>(yarpMatrix.rows()) != rows ||
        static_cast<size_t>(yarpMatrix.cols()) != cols )
    {
        return false;
    }

    // Here we explot the fact that both YARP and iDynTree store
    // Matrices in row major forma .
    memcpy(iDynTreeMatrix.data(),yarpMatrix.data(),rows*cols*sizeof(double));

    return true;
}


}

#endif
