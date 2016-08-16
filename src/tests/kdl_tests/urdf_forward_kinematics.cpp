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

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_export.hpp>

#include <kdl/tree.hpp>
#include "kdl_codyco/treefksolverpos_iterative.hpp"
#include <iostream>
#include <cstdlib>
#include <kdl/jntarray.hpp>
#include <ctime>
#include <cmath>

#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace std;
using namespace urdf;
using namespace KDL::CoDyCo;

void printFrame(TreeFkSolverPos_iterative & fwd_pos,
                GeneralizedJntPositions & pos,
                std::string frame_name)
{
    KDL::Frame out_frame;
    int ret = fwd_pos.JntToCart(pos,out_frame,frame_name);

    if( ret != 0 ) return;
    std::cout << frame_name << std::endl << out_frame << std::endl;
}

int main(int argc, char** argv)
{
    Tree my_tree, my_tree_converted;

    if (!iDynTree::treeFromUrdfFile(argv[1],my_tree))
    {
        cerr << "Could not generate robot model and extract kdl tree" << endl;
        return EXIT_FAILURE;
    }

    //Create forward kinematics object
    TreeFkSolverPos_iterative fwd_kin(my_tree);

    KDL::Frame world_root = KDL::Frame::Identity();
    KDL::JntArray q(fwd_kin.getUndirectedTree().getNrOfDOFs());
    SetToZero(q);

    q(fwd_kin.getUndirectedTree().getJunction("r_hip_pitch")->getDOFIndex()) = 0.0;
    q(fwd_kin.getUndirectedTree().getJunction("r_hip_roll")->getDOFIndex()) = 0.0;

    GeneralizedJntPositions floatPos(world_root,q);

    printFrame(fwd_kin,floatPos,"root_link");
    printFrame(fwd_kin,floatPos,"r_hip_1");
    printFrame(fwd_kin,floatPos,"r_hip_2");
    printFrame(fwd_kin,floatPos,"r_upper_thigh");


    return EXIT_SUCCESS;

}


