/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
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

/**
 * Convert a yarp::sig::Vector to a iDynTree::Wrench
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeWrench iDynTree::Wrench output
 * @return true if conversion was successful, false otherwise
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench);

/**
 * Convert a iDynTree::Wrench to a yarp::sig::Vector
 * @param iDynTreeWrench iDynTree::Wrench input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool toYarp(const iDynTree::Wrench & iDynTreeWrench, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector to a iDynTree::Position
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreePosition iDynTree::Position output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Position & iDynTreePosition);

/**
 * Convert a yarp::sig::Vector to a iDynTree::Vector3
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreePosition iDynTree::Vector3 output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Vector3 & iDynTreeVector3);

/**
 * Convert a iDynTree::Position to a yarp::sig::Vector of 3 elements.
 * @param iDynTreePosition iDynTree::Position input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was sucessful, false otherwise
 */
bool toYarp(const iDynTree::Position & iDynTreeWrench, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector of 3 elements to a iDynTree::Direction
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeDirection iDynTree::Direction output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 *
 * \note the direction vector will be normalized to have unit norm.
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Direction & iDynTreeDirection);

/**
 * Convert a iDynTree::Direction to a yarp::sig::Vector of 3 elements.
 * @param iDynTreeDirection iDynTree::Position input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was sucessful, false otherwise
 */
bool toYarp(const iDynTree::Vector3 & iDynTreeDirection, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector to a iDynTree::VectorDynSize
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeWrench iDynTree::VectorDynSize output
 * @return true if conversion was successful, false otherwise
 * \note the output VectorDynSize will be resized if necessary.
 */
bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::VectorDynSize & iDynTreeVector);

}

#endif
