/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
