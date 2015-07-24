/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015 Fondazione Istituto Italiano di Tecnologia
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

#include <cstdlib>

#include <fstream>
#include <sstream>

#include <ctime>
#include "kdl_format_io/iKin_export.hpp"

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

// KDL::Tree
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>


// KDL::CoDyCo::UndirectedTree
#include <kdl_codyco/undirectedtree.hpp>

// iCub::iKin::iKinChain
#include <iCub/iKin/iKinFwd.h>

using namespace KDL;
using namespace std;
using namespace iDynTree;

double random_double(double range)
{
    return range*((double)rand()-(RAND_MAX/2))/((double)RAND_MAX);
}

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


bool checkChainsAreEqual(KDL::Chain kdl_random_chain, iCub::iKin::iKinLimb & ikin_random_chain)
{
  srand(time(NULL));

  for(int i =0; i < 10; i++ )
  {
  if( kdl_random_chain.getNrOfJoints() != ikin_random_chain.getDOF() )
  {
      std::cerr << "urdf2dh: error in conversion, number of DOFs does not match" << std::endl;
      return false;
  }


  //Generate random state to validate
  KDL::JntArray q_kdl(kdl_random_chain.getNrOfJoints());
  yarp::sig::Vector q_yarp(ikin_random_chain.getDOF());

  for(int i=0; i < q_yarp.size(); i++ ) { q_kdl(i) = q_yarp(i) =  M_PI; /*0.0*random_double(2*M_PI);*/ }

  //Get H_ef_base for iKinChain
  ikin_random_chain.setAllConstraints(false);
  yarp::sig::Vector q_yarp_constrained = ikin_random_chain.setAng(q_yarp);
  yarp::sig::Matrix H_yarp = ikin_random_chain.getH();

  for(int i=0; i < q_yarp.size(); i++ ) { q_kdl(i) = q_yarp_constrained(i); }

  //Get H_ef_base for KDL::Chain
  KDL::Frame H_kdl;
  KDL::ChainFkSolverPos_recursive kdl_pos_solver(kdl_random_chain);
  kdl_pos_solver.JntToCart(q_kdl,H_kdl);


  yarp::sig::Matrix H_yarp_kdl = KDLtoYarp_position(H_kdl);

  //Check that the matrix are equal
  double tol = 1e-3;



  for(int i=0; i < 4; i++ ) {
      for(int j=0; j < 4; j++ ) {
          if( fabs(H_yarp_kdl(i,j)-H_yarp(i,j)) > tol )
          {
              std::cerr << "urdf2dh Element " << i << " " << j << " of the result matrix does not match" << std::endl;
              std::cerr << "H from iKinChain: " << std::endl << H_yarp.toString() << std::endl
                        << "H from KDL::Chain " << std::endl << H_yarp_kdl.toString() << std::endl;
              std::cerr << "Please open an issue at https://github.com/robotology-playground/idyntree/issues " << std::endl;
              return false;
          }

      }
  }
  }
  return true;
}

std::string int2string(int arg)
{
    std::ostringstream s;
    s << arg;
    return s.str();
}