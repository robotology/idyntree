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

bool KDLtoYarp(const KDL::Vector & kdlVector,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 3 ) { yarpVector.resize(3); }
    memcpy(yarpVector.data(),kdlVector.data,3*sizeof(double));
    return true;
}

bool KDLtoYarp(const KDL::Rotation & kdlRotation, yarp::sig::Matrix & yarpMatrix3_3)
{
    if( yarpMatrix3_3.rows() != 3 || yarpMatrix3_3.cols() != 3 ) { yarpMatrix3_3.resize(3,3); }
    //Both kdl and yarp store the rotation matrix in row major order
    memcpy(yarpMatrix3_3.data(),kdlRotation.data,3*3*sizeof(double));
    return true;
}
    
bool KDLtoYarp_position(const KDL::Frame & kdlFrame, yarp::sig::Matrix & yarpMatrix4_4 )
{
    yarp::sig::Matrix R(3,3);
    yarp::sig::Vector p(3);

    KDLtoYarp(kdlFrame.M,R);
    KDLtoYarp(kdlFrame.p,p);
    
    if( yarpMatrix4_4.rows() != 4 || yarpMatrix4_4.cols() != 4 ) { yarpMatrix4_4.resize(4,4); }
    yarpMatrix4_4.zero();
    
    yarpMatrix4_4.setSubmatrix(R,0,0);
    yarpMatrix4_4.setSubcol(p,0,3);
    yarpMatrix4_4(3,3) = 1;
    
    return true;
}    
    
yarp::sig::Matrix KDLtoYarp_position(const KDL::Frame & kdlFrame)
{
    yarp::sig::Matrix yarpMatrix4_4(4,4);
    KDLtoYarp_position(kdlFrame,yarpMatrix4_4);
    return yarpMatrix4_4;
}

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

KDL::Chain generateSimpleKDLChain(int nr_of_segments=1)
{
    KDL::Chain simple_chain;
    
    for(int i=0; i < nr_of_segments; i++ ) {
    double a = random_double(10);
    double d = random_double(10);
    double alpha = random_double(2*M_PI);
    double theta = random_double(2*M_PI);
    
    //double a = 10; //random_double(10);
    //double d = 0; //random_double(10);
    //double alpha = 0; //random_double(2*M_PI);
    //double theta = 0; //random_double(2*M_PI);
    
    std::cout << "Generated chain " << a << " " << d  << " " << alpha << " " << theta  << std::endl;
    std::cout << "cos(alpha) " << cos(alpha) << std::endl;
    std::cout << "sin(alpha) " << sin(alpha) << std::endl;
    KDL::Frame kdlFrame = KDL::Frame::DH(a,alpha,d,theta);
    simple_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),kdlFrame));
    }
    
    return simple_chain;
}

int main(int argc, char** argv)
{
  srand(time(NULL));
  
  //Generate random chain
  int nr_of_segments = 1;
  KDL::Chain kdl_random_chain = generateSimpleKDLChain(2);
  //KDL::Chain kdl_random_chain = generateRandomKDLChain(nr_of_segments,false,false);
  
  //Convert random KDL::Chain to iKinChain
  iCub::iKin::iKinChain ikin_random_chain;
  bool result = iKinChainFromKDLChain(kdl_random_chain,ikin_random_chain);
  
  #ifndef NDEBUG
  std::cout << "check_iKin_export_random_chain: converted chain" << std::endl;
  #endif
  
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
  //for(int i=0; i < ikin_random_chain.getN(); i++ ) { ikin_random_chain.releaseLink(i); }
  std::cout << "iKin_export_random_chain: Setting angles value in iKin" << std::endl; 
  ikin_random_chain.setAng(q_yarp);
  yarp::sig::Matrix H_yarp = ikin_random_chain.getH();
  
  yarp::sig::Matrix H_yarp_kdl = KDLtoYarp_position(H_kdl);
  
  //Check that the matrix are equal
  double tol = 1e-8;
  
  std::cout << "H_yarp" << std::endl << H_yarp.toString() << std::endl  << "H_kdl" << std::endl << H_yarp_kdl.toString() << std::endl << H_kdl << std::endl; 

  
  for(int i=0; i < 4; i++ ) {
      for(int j=0; j < 4; j++ ) {
          if( fabs(H_yarp_kdl(i,j)-H_yarp(i,j)) > tol ) { std::cerr << "Element " << i << " " << j << " of the result matrix does not match" << std::endl; return EXIT_FAILURE; }
      }
  }
 
 
  return EXIT_SUCCESS;
}


