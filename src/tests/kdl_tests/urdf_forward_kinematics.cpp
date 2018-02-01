/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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


