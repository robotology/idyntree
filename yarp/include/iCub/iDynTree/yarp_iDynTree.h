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
/*
#include <kdl/rotationalinertia.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
*/


/**
 * Convert a yarp::sig::Vector to a KDL::Wrench
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
bool iDynTreetoYarp(const iDynTree::Wrench & iDynTreeWrench,yarp::sig::Vector & yarpVector);

#endif
