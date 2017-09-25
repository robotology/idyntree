/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL2IDYN_H
#define KDL2IDYN_H


#include <iCub/iDyn/iDyn.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <cassert>



/**
 * Convert a KDL::Vector to a yarp::sig::Vector 
 * @param kdlVector KDL::Vector input
 * @param idynVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool to_iDyn(const KDL::Vector & kdlVector,yarp::sig::Vector & idynVector);

/**
 * 
 * @return true if conversion was successful, false otherwise
 */
bool kdlJntArray2idynVector(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & idynVector);


#endif
