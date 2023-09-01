// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_YARP_EIGEN_CONVERSIONS_H
#define IDYNTREE_YARP_EIGEN_CONVERSIONS_H

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

namespace iDynTree
{

/**
 * Convert a yarp::sig::Vector to a Eigen::Map<Eigen::VectorXd> object
 * @param yarpVector yarp::sig::Vector input
 * @return a Eigen::Map vector that points to the data contained in the yarp vector
 */
inline Eigen::Map<Eigen::VectorXd> toEigen(yarp::sig::Vector & yarpVector)
{
    return Eigen::Map<Eigen::VectorXd>(yarpVector.data(),yarpVector.size());
}

/**
 * Convert a yarp::sig::Matrix to a Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > object
 * @param yarpVector yarp::sig::Matrix input
 * @return a Eigen::Map vector that points to the data contained in the yarp matrix
 */
inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(yarp::sig::Matrix & yarpMatrix)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(yarpMatrix.data(),yarpMatrix.rows(),yarpMatrix.cols());
}

/**
 * Convert a const yarp::sig::Vector to a Eigen::Map<const Eigen::VectorXd> object
 * @param yarpVector yarp::sig::Vector input
 * @return a Eigen::Map vector that points to the data contained in the yarp vector
 */
inline Eigen::Map<const Eigen::VectorXd> toEigen(const yarp::sig::Vector & yarpVector)
{
    return Eigen::Map<const Eigen::VectorXd>(yarpVector.data(),yarpVector.size());
}

/**
 * Convert a const yarp::sig::Matrix to a Eigen::Map< const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > object
 * @param yarpVector yarp::sig::Matrix input
 * @return a Eigen::Map vector that points to the data contained in the yarp matrix
 */
inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const yarp::sig::Matrix & yarpMatrix)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(yarpMatrix.data(),yarpMatrix.rows(),yarpMatrix.cols());
}

}

#endif
