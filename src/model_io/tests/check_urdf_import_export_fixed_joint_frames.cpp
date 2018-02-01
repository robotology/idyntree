/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "kdl_format_io/urdf_import.hpp"
#include "kdl_format_io/urdf_export.hpp"
#include <kdl/tree.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <cstdlib>
#include <kdl/jntarray.hpp>
#include <ctime>
#include <cmath>

using namespace KDL;
using namespace std;
using namespace urdf;
using namespace KDL::CoDyCo;

void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has " << GetTreeElementChildren(link->second).size() << " children" << endl;
  for (unsigned int i=0; i<GetTreeElementChildren(link->second).size(); i++)
    printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
}


double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int main(int argc, char** argv)
{
    srand(time(NULL));

    Tree my_tree, my_tree_converted;
    if (!kdl_format_io::treeFromUrdfFile(argv[1],my_tree))
    {cerr << "Could not generate robot model and extract kdl tree" << endl; return EXIT_FAILURE;}

    // walk through tree
    cout << " ======================================" << endl;
    cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
    cout << " ======================================" << endl;
    SegmentMap::const_iterator root = my_tree.getRootSegment();
    printLink(root, "");

    //Export current tree
    std::string output_name = "test_kdl_format_io.urdf";
    if( !kdl_format_io::treeToUrdfFile(output_name,my_tree) )
    {cerr <<"Could not generate urdf from kdl tree" << endl; return EXIT_FAILURE;}

    //Re-importing it
    if( !kdl_format_io::treeFromUrdfFile(output_name,my_tree_converted) )
    {cerr <<"Could not re-import back generated urdf file" << endl; return EXIT_FAILURE;}

    //Preliminary test
    if( my_tree.getNrOfJoints() != my_tree_converted.getNrOfJoints() ) {
        cerr << "Error in conversion " << std::endl;
        return EXIT_FAILURE;
    }

    //Running inverse dynamics for being sure all went well
    TreeIdSolver_RNE original_slv(my_tree), converted_slv(my_tree_converted);


    JntArray q,dq,ddq,torques,torques_converted;
    std::vector<Wrench> f,f_ext;
    Wrench base_force, base_force_converted;
    Twist base_vel, base_acc;

    q = dq = ddq = torques = torques_converted = JntArray(my_tree.getNrOfJoints());
    f = f_ext = std::vector<Wrench>(my_tree.getNrOfSegments(),KDL::Wrench::Zero());

    for(int i=0; i < my_tree.getNrOfJoints(); i++ )
    {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }

    base_vel = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    base_acc = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));

    //Inserting the random input data in both solvers, while checking all went well
    if( original_slv.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques,base_force) != 0 )
    { std::cerr << "Could not load solver for original tree" << std::endl; return EXIT_FAILURE; }
    if( converted_slv.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques_converted,base_force_converted) != 0 )
    { std::cerr << "Could not load solver for converted tree" << std::endl; return EXIT_FAILURE; }

    double tol = 1e-4;
    for( int i=0; i < my_tree.getNrOfJoints(); i++ )
    {
        std::cout << fabs(torques(i)-torques_converted(i)) << std::endl;
        if( fabs(torques(i)-torques_converted(i)) > tol ) return -1;
    }

    for( int i=0; i < 6; i++ ) {
        std::cout << fabs(base_force(i)-base_force_converted(i)) << std::endl;
        if( fabs(base_force(i)-base_force_converted(i)) > tol ) return -1;
    }


    return EXIT_SUCCESS;

}


