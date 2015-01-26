/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN2KDL_H
#define IDYN2KDL_H

#include <iCub/iDyn/iDyn.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDynInv.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <cassert>

using namespace yarp::math;

/**
 * Convert an iCub::iDyn::iDynChain object to a KDL::Chain object
 * @param idynChain iDynChain object input
 * @param kdlChain KDL::Chain object output
 * @param link_names vector of the names of the links
 * @param joint_names vector of the names of the joints
 * @param final_frame_name if specified, the HN transformation is explicitly added as a virtual fixed link 
 * @param initial_frame_name if specified, the H0 transformation is explicitly added as a virtual fixed link 
 * @return true if conversion was successful, false otherwise
 */
bool idynChain2kdlChain(iCub::iDyn::iDynChain & idynChain,
                        KDL::Chain & kdlChain,
                        std::vector<std::string> link_names = std::vector<std::string>(0),
                        std::vector<std::string> joint_names = std::vector<std::string>(0),
                        std::string final_frame_name = "",
                        std::string initial_frame_name = "",
                        int max_links = 10000);

/**
 * Convert an iCub::iDyn::iDynChain object together with an iCub::iDynInvSensor to a KDL::Chain object,
 * If idynChain has N links, the create kdlChain has N+1 links, with the sensor link divided by the sensor in two semilinks
 * @param idynChain iCub::iDyn::iDynChain object input
 * @param kdlChain KDL::Chain object output
 * @param link_names vector of the names of the links
 * @param joint_names vector of the names of the joints
 * @param final_frame_name if specified, the HN transformation is explicitly added as a virtual fixed link 
 * @param initial_frame_name if specified, the H0 transformation is explicitly added as a virtual fixed link 
 * @return true if conversion was successful, false otherwise
 */
bool idynSensorChain2kdlChain(iCub::iDyn::iDynChain & idynChain,
                              iCub::iDyn::iDynInvSensor & idynSensor,
                              KDL::Chain & kdlChain, 
                              std::vector<std::string> link_names = std::vector<std::string>(0),
                              std::vector<std::string> joint_names = std::vector<std::string>(0),
                              std::string final_frame_name = "",
                              std::string initial_frame_name = "",
                              int max_links = 10000);


//bool idynLink2kdlSegment(const iCub::iDyn::iDynLink & idynLink,KDL::Segment & kdlSegment);

/**
 * Convert the iCub::iDyn::iDynLink dynamical parameters to an KDL::RigidBodyInertia
 * @param mass mass of the iDynLink object input
 * @param rC COM of the iDynLink object input
 * @param I Inertia of the iDynLink object input
 * @param kdlRigidBodyInertia KDL::RigidBodyInertia object output
 * @return true if conversion was successful, false otherwise
 */
bool idynDynamicalParameters2kdlRigidBodyInertia(const double mass,const yarp::sig::Vector & rC,const yarp::sig::Matrix & I,KDL::RigidBodyInertia & kdlRigidBodyInertia);  

/**
 * Convert a 3x3 yarp::sig::Matrix representing the Inertia matrix of the iDynLink object, to a KDL::RotationalInertia object
 * @param idynInertia Inertia of the iDynLink object input
 * @param kdlRotationalInertia KDL::RigidBodyInertia object output
 * @return true if conversion was successful, false otherwise
 */
bool idynInertia2kdlRotationalInertia(const yarp::sig::Matrix & idynInertia,KDL::RotationalInertia & kdlRotationalInertia);

/**
 * Convert a 4x4 yarp::sig::Matrix rapresenting a rototranslation matrix (element of SE(3)), to a KDL::Frame object
 * @param idynMatrix yarp::sig::Matrix input
 * @param kdlFrame KDL::Frame output
 * @return true if conversion was successful, false otherwise
 */
bool idynMatrix2kdlFrame(const yarp::sig::Matrix & idynMatrix, KDL::Frame & kdlFrame);

/**
 * Convert a 3x3 yarp::sig::Matrix rapresenting a rotation matrix (element of SO(3)), to a KDL::Rotation object
 * @param idynMatrix yarp::sig::Matrix input
 * @param kdlRotation KDL::Rotation output
 * @return true if conversion was successful, false otherwise
 */
bool idynMatrix2kdlRotation(const yarp::sig::Matrix & idynMatrix, KDL::Rotation & kdlRotation);

/**
 * Convert a yarp::sig::Vector to a KDL::Vector
 * @param idynVector yarp::sig::Vector input
 * @param kdlVector KDL::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool idynVector2kdlVector(const yarp::sig::Vector & idynVector, KDL::Vector & kdlVector);


void printKDLchain(std::string s,const KDL::Chain & kldChain);


/**
 * Modify a KDL::Chain by transforming the base link reference frame
 *
 * @param old_chain the input chain
 * @param new_chain the output chain
 * @param H_new_old tranformation from the old base reference frame to the new one ({}^{new}H_old)
 * @return true if conversion was successful, false otherwise
 * 
 */ 
bool addBaseTransformation(const KDL::Chain & old_chain, KDL::Chain & new_chain, KDL::Frame H_new_old);

/**
*
* Returns the inverse of a 4 by 4 rototranslational matrix 
* @param H is the 4 by 4 rototranslational matrix.
* @param verbose sets some verbosity.  
* @return inverse of 4 by 4 rototranslational matrix. 
*  
* @note about 5 times faster than pinv() 
*/
yarp::sig::Matrix localSE3inv(const yarp::sig::Matrix &H, unsigned int verbose=0);

//yarp::sig::Matrix invert_SE3_matrix(const yarp::sig::Matrix & H);
#endif
