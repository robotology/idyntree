/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2013 Istituto Italiano di Tecnologia
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Silvio Traversaro */

#include <kdl_format_io/iKin_export.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace std;
using namespace kdl_format_io;

double random_double(double range)
{
    return range*((double)rand()-(RAND_MAX/2))/((double)RAND_MAX);
}

int random_integer(int range)
{
    return rand()%range;
}

KDL::Joint generateRandomKDLJoint(bool use_translational_joints, bool use_fixed_joints)
{
    KDL::Joint::JointType random_joint_type;
    int nr_of_choices = 3;
    if( use_translational_joints ) { nr_of_choices += 3; }
    if( use_fixed_joints ) { nr_of_choices++; }
    switch(random_integer(nr_of_choices)) 
    {
        case 0:
            random_joint_type = Joint::RotX;
        break;
        case 1:
            random_joint_type = Joint::RotY;
        break;
        case 2:
            random_joint_type = Joint::RotZ;
        break;
        case 3:
            if( nr_of_choices == 4 ) {
                random_joint_type = Joint::None;
            } else {
                random_joint_type = Joint::TransX;
            }
        break;
        case 4:
            random_joint_type = Joint::TransY;
        break;
        case 5:
            random_joint_type = Joint::TransZ;
        break;
        case 6:
            random_joint_type = Joint::None;
        break;
        default:
            random_joint_type = Joint::None;
        break;
    }
    return KDL::Joint(random_joint_type);
}

KDL::Frame generateRandomKDLFrame()
{
    KDL::Rotation random_rotation = KDL::Rotation::EulerZYZ(random_double(2*M_PI),random_double(2*M_PI),random_double(2*M_PI));
    KDL::Vector random_vector = KDL::Vector(random_double(10),random_double(10),random_double(10));
    
    return KDL::Frame(random_rotation,random_vector);
}

KDL::Chain generateRandomKDLChain(int nr_of_segments, bool use_translational_joints = true, bool use_fixed_joints = true)
{
    KDL::Chain random_chain;
    for(int i=0; i < nr_of_segments; i++ ) {
        random_chain.addSegment(KDL::Segment(generateRandomKDLJoint(use_translational_joints,use_fixed_joints),generateRandomKDLFrame()));
    }
    return random_chain;
}

int main(int argc, char** argv)
{
  srand(time(NULL));
 
  //Generate random chain
  int nr_of_segments = 20;
  KDL::Chain kdl_random_chain = generateRandomKDLChain(nr_of_segments,false,false);
  
  //Convert random KDL::Chain to iKinChain
  iCub::iKin::iKinChain ikin_random_chain;
  bool result = iKinChainFromKDLChain(kdl_random_chain,ikin_random_chain);
  
  if( !result) { std::cerr << "Error in KDL - iKin conversion" << std::endl; return EXIT_FAILURE; }
  
  //Generate random state to validate  
  KDL::JntArray q_kdl(kdl_random_chain.getNrOfJoints());
  yarp::sig::Vector q_yarp(ikin_random_chain.getDOF());
  
  if( kdl_random_chain.getNrOfJoints() != ikin_random_chain.getDOF() ) { std::cerr << "The number of DOFs of the KDL::Chain and the iKinChain does not match " << std::endl; return EXIT_FAILURE; }
  
  for(int i=0; i < q_yarp.size(); i++ ) { q_kdl(i) = q_yarp(i) = random_double(2*M_PI); } 
  
  //Get H_ef_base for KDL::Chain
  KDL::Frame H_kdl;
  KDL::ChainFkSolverPos_recursive kdl_pos_solver(kdl_random_chain);
  kdl_pos_solver.JntToCart(q_kdl,H_kdl);
  
  //Get H_ef_base for iKinChain
  for(int i=0; i < ikin_random_chain.getN(); i++ ) { ikin_random_chain.releaseLink(i); }
  std::cout << "iKin_export_random_chain: Setting angles value in iKin" << std::endl; 
  ikin_random_chain.setAng(q_yarp);
  yarp::sig::Matrix H_yarp = ikin_random_chain.getH();
  
  //Check that the matrix are equal
  double tol = 1e-8;
  
  for(int i=0; i < 4; i++ ) {
      for(int j=0; j < 3; j++ ) {
          if( fabs(H_kdl(i,j)-H_yarp(i,j)) > tol ) { std::cerr << "Element " << i << " " << j << "of the result matrix does not match" << std::endl; return EXIT_FAILURE; }
      }
  }
 
  return EXIT_SUCCESS;
}


