/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include "../../tools/urdf2dh_helpers.h"

int main(int argc, char** argv)
{
  std::string urdf_file_name         = argv[1];

  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  iCub::iKin::iKinLimb ikin_limb;

  //
  // URDF --> KDL::Tree
  //
  bool root_inertia_workaround = true;
  if( !treeFromUrdfFile(urdf_file_name,kdl_tree,root_inertia_workaround) )
  {
      cerr << "urdf2dh_tests: Could not parse urdf robot model" << endl;
      std::cerr << "urdf2dh_tests: Please open an issue at https://github.com/robotology-playground/idyntree/issues " << std::endl;

      return EXIT_FAILURE;
  }

  //
  // KDL::Tree --> KDL::CoDyCo::UndirectedTree
  // (for extracting arbitrary chains,
  //    using KDL::Tree you can just get chains where the base of the chain
  //    is proximal to the tree base with respect to the end effector.
  //
  KDL::CoDyCo::UndirectedTree undirected_tree(kdl_tree);

  std::string base_frame;
  std::string end_effector_frame;

  bool test_success = true;
  int success_count = 0;
  int failure_count = 0;

  std::vector<int> failure_as_base_frame(undirected_tree.getNrOfLinks(),0);
  std::vector<int> failure_as_ee_frame(undirected_tree.getNrOfLinks(),0);
  std::vector<int> success_as_base_frame(undirected_tree.getNrOfLinks(),0);
  std::vector<int> success_as_ee_frame(undirected_tree.getNrOfLinks(),0);

  for(int base_frame_index = 0;
      base_frame_index < undirected_tree.getNrOfLinks();
      base_frame_index++ )
  {
      for(int end_effector_frame_index = 0;
          end_effector_frame_index < undirected_tree.getNrOfLinks();
          end_effector_frame_index++ )
      {
            if( base_frame_index == end_effector_frame_index )
            {
                continue;
            }

            base_frame = undirected_tree.getLink(base_frame_index)->getName();
            end_effector_frame = undirected_tree.getLink(end_effector_frame_index)->getName();

            // There is a bug in undirected_tree.getChain("","base_link"), disregarding does result for now
            if( base_frame == "base_link" ||
                end_effector_frame == "base_link" )
            {
                continue;
            }


            //cerr << "urdf2dh test testing chain extraction between " << base_frame
            //     << " and " << end_effector_frame << endl;

            KDL::Tree kdl_rotated_tree = undirected_tree.getTree(base_frame);

            bool result = kdl_rotated_tree.getChain(base_frame,end_effector_frame,kdl_chain);
            if( !result )
            {
                cerr << "urdf2dh: Impossible to find " << base_frame << " or "
                    << end_effector_frame << " in the URDF."  << endl;
                return EXIT_FAILURE;
            }


        //
        // Convert the chain and the limits to an iKin chain (i.e. DH parameters)
        //
        KDL::JntArray qmin(kdl_chain.getNrOfJoints()),qmax(kdl_chain.getNrOfJoints());

        for(int i=0; i < qmin.rows(); i++ ) { qmin(i) = -10.0; qmax(i) = 10.0; }

        result = iKinLimbFromKDLChain(kdl_chain,ikin_limb,qmin,qmax);
        if( !result )
        {
            cerr << "urdf2dh: Could not export KDL::Tree to iKinChain" << endl;
            return EXIT_FAILURE;
        }

        bool result_corrupted = false;
        if( !checkChainsAreEqual(kdl_chain,ikin_limb) )
        {
            cerr << "urdf2dh_test error: KDL::Chain and iKinChain between "
                 << base_frame << " and " << end_effector_frame << " results does not match" << endl;
            test_success = false;
            failure_count++;
            failure_as_base_frame[base_frame_index]++;
            failure_as_ee_frame[end_effector_frame_index]++;
        }
        else
        {
            success_count++;
            success_as_base_frame[base_frame_index]++;
            success_as_ee_frame[end_effector_frame_index]++;
        }

     }
  }

  cerr << "urdf2dh report: " << endl;
  for(int i=0; i < undirected_tree.getNrOfLinks(); i++ )
  {
      std::string frame= undirected_tree.getLink(i)->getName();
      cerr << frame  << " as base frame   : " << endl;
      cerr << "\t total frame couples failed : " << failure_as_base_frame[i] << endl;
      cerr << frame  << " as ee frame   : " << endl;
      cerr << "\t total frame couples failed : " << failure_as_ee_frame[i] << endl;
  }

  cerr << "total frame couples tested : " << failure_count + success_count << endl;
  cerr << "total frame couples failed : " << failure_count << endl;
  cerr << "total frame couples success : " << success_count << endl;

  return test_success;
}


