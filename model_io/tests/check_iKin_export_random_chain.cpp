/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015 Istituto Italiano di Tecnologia
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
*   * Neither the name of the Istituto Italiano di Tecnologia nor the names of its
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

#include "kdl_format_io/iKin_export.hpp"

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

double random_positive_double(double range)
{
    return range*((double)rand())/((double)RAND_MAX);
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

KDL::Chain generateSimpleKDLChain(int nr_of_segments=1,
                                  double a_gain = 1.0,
                       double d_gain = 1.0,
                       double alpha_gain = 1.0,
                       double offset_gain = 1.0)
{
    KDL::Chain simple_chain;

    for(int i=0; i < nr_of_segments; i++ ) {
    double a = a_gain*random_double(10);
    double d = d_gain*random_double(10);
    double alpha = alpha_gain*random_double(2*M_PI);
    double theta = offset_gain*random_double(2*M_PI);

    //double a = 10; //random_double(10);
    //double d = 0; //random_double(10);
    //double alpha = 0; //random_double(2*M_PI);
    //double theta = 0; //random_double(2*M_PI);

    //std::cout << "Generated chain " << a << " " << d  << " " << alpha << " " << theta  << std::endl;
    /*
    std::cout << "cos(alpha) " << cos(alpha) << std::endl;
    std::cout << "sin(alpha) " << sin(alpha) << std::endl;
    */
    KDL::Frame kdlFrame = KDL::Frame::DH(a,alpha,d,theta);
    simple_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),kdlFrame));
    }

    return simple_chain;
}


int checkRandomKDLtoDH(int nr_of_joints,
                       std::string chain_type,
                       double a_gain = 1.0,
                       double d_gain = 1.0,
                       double alpha_gain = 1.0,
                       double offset_gain = 1.0)
{
  //Generate    random chain
  KDL::Chain kdl_random_chain;
  if( chain_type == "dh" )
  {
      kdl_random_chain = generateSimpleKDLChain(nr_of_joints,a_gain,d_gain,alpha_gain,offset_gain);
  }
  else if( chain_type == "random")
  {
      kdl_random_chain = generateRandomKDLChain(nr_of_joints,false,false);
  }
  else
  {
      std::cout << "chain_type not supported" << std::endl;
      return 0;
  }

  //Convert random KDL::Chain to iKinChain
  iCub::iKin::iKinLimb ikin_random_limb;
  KDL::JntArray max(kdl_random_chain.getNrOfJoints());
  KDL::JntArray min(kdl_random_chain.getNrOfJoints());

  bool result = kdl_format_io::iKinLimbFromKDLChain(kdl_random_chain,ikin_random_limb,min,max);


  iCub::iKin::iKinChain * p_ikin_random_chain = ikin_random_limb.asChain();
  iCub::iKin::iKinChain & ikin_random_chain = *p_ikin_random_chain;

  std::cout << "check_iKin_export_random_chain: converted chain" << std::endl;
  std::cout << ikin_random_chain.getH0().toString() << std::endl;
  for(int i=0; i < ikin_random_chain.getDOF(); i++ )
  {
    std::cout << ikin_random_chain[i].getA() << " " << ikin_random_chain[i].getD() << " " <<
                 ikin_random_chain[i].getAlpha() << " " << ikin_random_chain[i].getOffset() << std::endl;
  }
  std::cout << ikin_random_chain.getHN().toString() << std::endl;


  if( !result) { std::cerr << "Error in KDL - iKin conversion" << std::endl; return 0; }

  //Generate random state to validate
  KDL::JntArray q_kdl(kdl_random_chain.getNrOfJoints());
  yarp::sig::Vector q_yarp(ikin_random_chain.getDOF());

  if( kdl_random_chain.getNrOfJoints() != ikin_random_chain.getDOF() ) { std::cerr << "The number of DOFs of the KDL::Chain and the iKinChain does not match " << std::endl; return 0; }

  //srand(time(0));
  for(int i=0; i < q_yarp.size(); i++ ) { q_kdl(i) = q_yarp(i) = 1*random_double(2*M_PI); }

  //Get H_ef_base for KDL::Chain
  KDL::Frame H_kdl;
  KDL::ChainFkSolverPos_recursive kdl_pos_solver(kdl_random_chain);
  kdl_pos_solver.JntToCart(q_kdl,H_kdl);

  //Get H_ef_base for iKinChain
  for(int i=0; i < ikin_random_chain.getN(); i++ ) { ikin_random_chain.releaseLink(i); }
  //do not enfore limits while checking the model
  ikin_random_chain.setAllConstraints(false);
  //std::cout << "iKin_export_random_chain: Setting angles value in iKin" << std::endl;
  std::cout << q_yarp.toString() << std::endl;
  q_yarp = ikin_random_chain.setAng(q_yarp);
  std::cout << q_yarp.toString() << std::endl;
  yarp::sig::Matrix H_yarp = ikin_random_chain.getH();

  yarp::sig::Matrix H_yarp_kdl = KDLtoYarp_position(H_kdl);

  //Check that the matrix are equal
  double tol = 1e-2;



  for(int i=0; i < 4; i++ ) {
      for(int j=0; j < 4; j++ ) {
          if( fabs(H_yarp_kdl(i,j)-H_yarp(i,j)) > tol )
          {
              std::cerr << "Element " << i << " " << j << " of the result matrix does not match" << std::endl;
              std::cout << "H_yarp" << std::endl << H_yarp.toString() << std::endl  << "H_kdl" << std::endl << H_yarp_kdl.toString() << std::endl;
              return 0;
          }
      }
  }

  return 1;
}

//List of tests to do:
//1 dof: dh generated random d,a,alpha,theta separated, couples, triple, all of them
//2 dof

int main(int argc, char** argv)
{
    //srand(time(0));
    srand(0);

    // \todo TODO : test fails for dof ~= 100 , check why and fix
    for(int i=0;i<10;i++)
    {
        for(int permutation=0; permutation < 16; permutation++ )
        {
            double a_gain = (permutation % 2)/1 == 0 ? 0.0 : 1.0;
            double d_gain = (permutation % 4)/2 == 0 ? 0.0 : 1.0;
            double alpha_gain = (permutation % 8)/4 == 0 ? 0.0 : 1.0;
            double offset_gain = (permutation % 16)/8 == 0 ? 0.0 : 1.0;

            if( !checkRandomKDLtoDH(i,"dh",a_gain,d_gain,alpha_gain,1.0*offset_gain) )
            {
                std::cout << "DH to DH conversion failing for: " << std::endl;
                std::cout << i << " " << a_gain << " " << d_gain << " " << alpha_gain << " " << offset_gain << std::endl;
                return EXIT_FAILURE;
            }
            else
            {
                std::cout << "DH to DH conversion successfull for: " << std::endl;
                std::cout << i << " " << a_gain << " " << d_gain << " " << alpha_gain << " " << offset_gain << std::endl;
            }
        }
    }

    for(int i=0;i<10;i++)
    {

            if( !checkRandomKDLtoDH(i,"random") )
            {
                std::cout << "Random to DH conversion failing for nr_of_joints = " << i << std::endl;
                return EXIT_FAILURE;
            }
    }

    std::cout << "All DH conversion test completed successfully." << std::endl;
    return EXIT_SUCCESS;
}


