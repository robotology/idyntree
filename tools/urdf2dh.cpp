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

#include "kdl_format_io/urdf_import.hpp"

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
using namespace kdl_format_io;

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
  srand(0);

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

  for(int i=0; i < q_yarp.size(); i++ ) { q_kdl(i) = q_yarp(i) = 1*random_double(2*M_PI); }

  //Get H_ef_base for iKinChain
  //for(int i=0; i < ikin_random_chain.getN(); i++ ) { ikin_random_chain.releaseLink(i); }
  //std::cout << "iKin_export_random_chain: Setting angles value in iKin" << std::endl;
  //std::cout << q_yarp.toString() << std::endl;
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

void printHelp()
{
    std::cerr << "URDF to Denavit-Hartenberg parameter converter." << std::endl;
    std::cerr << "This utility will extract any chain, and it will output" << std::endl;
    std::cerr << "the Denavit-Hartenberg parameters in iKin .ini format." << std::endl;
    std::cerr << "For more information about the iKin .ini format, check:" << std::endl;
    std::cerr << "      http://wiki.icub.org/iCub/main/dox/html/classiCub_1_1iKin_1_1iKinLimb.html#a76c93aae76bb0f7ef9470b81d0da0e26" << std::endl;
    std::cerr << "Usage: urdf2dh robot.urdf base_link_name end_effector_link_name dhParams.ini" << std::endl;
}

std::string int2string(int arg)
{
    std::ostringstream s;
    s << arg;
    return s.str();
}

int main(int argc, char** argv)
{
  if ( argc == 2 )
  {
      if( (std::string(argv[1]) == "--help" ||
           std::string(argv[1]) == "-h" ) )
      {
          printHelp();
          return 0;
      }
  }
  if (argc != 5)
  {
      printHelp();
      return -1;
  }

  std::string urdf_file_name         = argv[1];
  std::string base_link_name         = argv[2];
  std::string end_effector_link_name = argv[3];
  std::string ikin_ini_file_name     = argv[4];

  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  iCub::iKin::iKinLimb ikin_limb;
  std::vector<std::string> joint_names;
  KDL::JntArray min,max;

  //
  // URDF --> KDL::Tree
  //
  bool root_inertia_workaround = true;
  if( !treeFromUrdfFile(urdf_file_name,kdl_tree,root_inertia_workaround) )
  {
      cerr << "urdf2dh: Could not parse urdf robot model" << endl;
      std::cerr << "urdf2dh: Please open an issue at https://github.com/robotology-playground/idyntree/issues " << std::endl;

      return EXIT_FAILURE;
  }

  //
  // URDF --> position ranges
  //
  if( !jointPosLimitsFromUrdfFile(urdf_file_name,joint_names,min,max) )
  {
      cerr << "Could not parse urdf robot model limits" << endl;
      return EXIT_FAILURE;
  }

  if( joint_names.size() != min.rows() ||
      joint_names.size() != max.rows() ||
      joint_names.size() == 0)
  {
      cerr << "Inconsistent joint limits got from urdf (nr of joints extracted: " << joint_names.size() << " ) " << endl;
      return EXIT_FAILURE;
  }

  //
  // KDL::Tree --> KDL::CoDyCo::UndirectedTree
  // (for extracting arbitrary chains,
  //    using KDL::Tree you can just get chains where the base of the chain
  //    is proximal to the tree base with respect to the end effector.
  //
  KDL::CoDyCo::UndirectedTree undirected_tree(kdl_tree);

  KDL::Tree kdl_rotated_tree = undirected_tree.getTree(base_link_name);

  bool result = kdl_rotated_tree.getChain(base_link_name,end_effector_link_name,kdl_chain);
  if( !result )
  {
      cerr << "urdf2dh: Impossible to find " << base_link_name << " or "
           << end_effector_link_name << " in the URDF."  << endl;
      return EXIT_FAILURE;
  }

  //
  // Copy the limits extracted from the URDF to the chain
  //
  int nj = kdl_chain.getNrOfJoints();
  KDL::JntArray chain_min(nj), chain_max(nj);

  int seg_i, jnt_i;
  for(seg_i=0,jnt_i=0; seg_i < kdl_chain.getNrOfSegments(); seg_i++)
  {
      const Segment & seg = kdl_chain.getSegment(seg_i);
      if( seg.getJoint().getType() != KDL::Joint::None )
      {
          std::string jnt_name = seg.getJoint().getName();
         // std::cerr << "searching for joint " << jnt_name << std::endl;
          int tree_jnt = 0;
          for(tree_jnt = 0; tree_jnt < joint_names.size(); tree_jnt++ )
          {
              //std::cerr << "joint_names[ " << tree_jnt << "] is " << joint_names[tree_jnt] << std::endl;
              if( joint_names[tree_jnt] == jnt_name )
              {
                  chain_min(jnt_i) = min(tree_jnt);
                  chain_max(jnt_i) = max(tree_jnt);
                  jnt_i++;
                  break;
              }
          }
          if( tree_jnt == joint_names.size() )
          {
              std::cerr << "urdf2dh failure in converting limits from tree to chain, unable to find joint " << jnt_name << std::endl;
              return EXIT_FAILURE;
          }
      }
  }

  if( jnt_i != nj )
  {
      std::cerr << "urdf2dh failure in converting limits from tree to chain" << std::endl;
      return EXIT_FAILURE;
  }

  //
  // Convert the chain and the limits to an iKin chain (i.e. DH parameters)
  //
  result = iKinLimbFromKDLChain(kdl_chain,ikin_limb,chain_min,chain_max);
  if( !result )
  {
      cerr << "urdf2dh: Could not export KDL::Tree to iKinChain" << endl;
      return EXIT_FAILURE;
  }

  bool result_corrupted = false;
  if( !checkChainsAreEqual(kdl_chain,ikin_limb) )
  {
      cerr << "urdf2dh error: KDL::Chain and iKinChain results does not match" << endl;
      std::cerr << "urdf2dh: Please open an issue at https://github.com/robotology-playground/idyntree/issues " << std::endl;
      return false;
  }

  yarp::os::Property prop;
  result = ikin_limb.toLinksProperties(prop);
  if( !result )
  {
      cerr << "urdf2dh: Could not export Link Properties from ikin_limb" << endl;
      return EXIT_FAILURE;
  } else {
      std::cout << "urdf2dh: Conversion to iKin DH chain completed correctly" << std::endl;
  }

  std::string ikin_prop = prop.toString();
  yarp::os::Bottle prop_bot;
  prop_bot.fromString(prop.toString());

  //Write the properties to file
   std::ofstream ofs (ikin_ini_file_name.c_str(), std::ofstream::out);

   ofs << prop_bot.findGroup("type").toString() << std::endl;
   ofs << prop_bot.findGroup("numLinks").toString() << std::endl;
   ofs << prop_bot.findGroup("H0").toString() << std::endl;
   for( int link = 0; link < ikin_limb.getN(); link++ )
   {
        std::string link_name = "link_" + int2string(link);
        ofs << prop_bot.findGroup(link_name).toString() << std::endl;

   }
   ofs << prop_bot.findGroup("HN").toString() << std::endl;


   ofs.close();

   if( result_corrupted )
   {
       return EXIT_FAILURE;
   }
   else
   {
     return EXIT_SUCCESS;
   }
}


