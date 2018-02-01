/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "testModels.h"

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_export.hpp>

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

std::string getDOFName(KDL::Tree & tree, const unsigned int index)
{
    std::string retVal = "";
    for (KDL::SegmentMap::const_iterator it=tree.getSegments().begin(); it!=tree.getSegments().end(); ++it)
    {
        if( GetTreeElementQNr(it->second) == index )
        {
            retVal = GetTreeElementSegment(it->second).getJoint().getName();
        }
    }

    return retVal;
}

bool checkUrdfImportExport(std::string urdfFileName)
{
    cout << "------> Testing file " << urdfFileName << std::endl;


    Tree my_tree, my_tree_converted;
    if (!iDynTree::treeFromUrdfFile(urdfFileName,my_tree))
    {cerr << "Could not generate robot model and extract kdl tree" << endl; return false;}


    //Export current tree
    cout << "Writing KDL::Tree to a urdf file" << endl;
    std::string output_name = "test_kdl_format_io.urdf";
    if( !iDynTree::treeToUrdfFile(output_name,my_tree) )
    {cerr <<"Could not generate urdf from kdl tree" << endl; return false;}

    //Re-importing it
    cout << "Reimporting  written urdf file" << endl;
    if( !iDynTree::treeFromUrdfFile(output_name,my_tree_converted) )
    {cerr <<"Could not re-import back generated urdf file" << endl; return false;}

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

    for(size_t i=0; i < my_tree.getNrOfJoints(); i++ )
    {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }

    base_vel = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    base_acc = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));

    //Inserting the random input data in both solvers, while checking all went well
    if( original_slv.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques,base_force) != 0 )
    { std::cerr << "Could not load solver for original tree" << std::endl; return false; }
    if( converted_slv.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques_converted,base_force_converted) != 0 )
    { std::cerr << "Could not load solver for converted tree" << std::endl; return false; }

    double tol = 1e-1;
    bool testFailed = false;
    for( size_t i=0; i < my_tree.getNrOfJoints(); i++ )
    {
        if( fabs(torques(i)-torques_converted(i)) > tol )
        {
            std::cerr << "Error in compute torque of joint " << getDOFName(my_tree,i) << std::endl;
            std::cerr << "Original " << torques(i) << " converted " << torques_converted(i)
                      << " error " << fabs(torques(i)-torques_converted(i)) << std::endl;
            testFailed = true;
        }
    }

    for( int i=0; i < 6; i++ )
    {
        if( fabs(base_force(i)-base_force_converted(i)) > tol )
        {
            std::cerr << "Error in element " << i << "of computed base ft" << std::endl;
            std::cerr << "Original " << base_force(i) << " converted " << base_force_converted(i)
                      << " error " << fabs(base_force(i)-base_force_converted(i)) << std::endl;
            testFailed = true;
        }
    }

    return !testFailed;
}

int main(int argc, char** argv)
{
    srand(time(NULL));

    bool testSuccess = true;


    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = std::string(IDYNTREE_TEST_MODELS_PATH) + "/" +
                                   std::string(IDYNTREE_TESTS_URDFS[mdl]);

        // This test fails with frame.urdf, see https://github.com/robotology/idyntree/issues/298
        if (std::string(IDYNTREE_TESTS_URDFS[mdl]) != "frame.urdf")
        {
            bool ok = checkUrdfImportExport(urdfFileName);
            testSuccess = testSuccess && ok;
        }
    }

    return testSuccess ? EXIT_SUCCESS : EXIT_FAILURE;

}


