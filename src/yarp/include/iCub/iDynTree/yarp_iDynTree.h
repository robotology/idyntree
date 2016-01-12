/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#ifndef YARP_IDYNTREE_H
#define YARP_IDYNTREE_H

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/Core/Wrench.h>

/**
 * Convert a yarp::sig::Vector to a iDynTree::Wrench
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreeWrench iDynTree::Wrench output
 * @return true if conversion was successful, false otherwise
 */
bool YarptoiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench);

/**
 * Convert a iDynTree::Wrench to a yarp::sig::Vector
 * @param iDynTreeWrench iDynTree::Wrench input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool iDynTreetoYarp(const iDynTree::Wrench & iDynTreeWrench, yarp::sig::Vector & yarpVector);

/**
 * Convert a yarp::sig::Vector to a iDynTree::Position
 * @param yarpVector yarp::sig::Vector input
 * @param iDynTreePosition iDynTree::Position output
 * @return true if conversion was successful, false otherwise (if the input yarpVector has size different from 3)
 */
bool YarptoiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Position & iDynTreePosition);


/**
 * Convert a iDynTree::Position to a yarp::sig::Vector of 3 elements.
 * @param iDynTreePosition iDynTree::Position input
 * @param yarpVector yarp::sig::Vector output
 * @return true if conversion was sucessful, false otherwise
 */
bool iDynTreetoYarp(const iDynTree::Position & iDynTreeWrench, yarp::sig::Vector & yarpVector);

#endif
