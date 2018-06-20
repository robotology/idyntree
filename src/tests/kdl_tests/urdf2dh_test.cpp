/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "../../tools/urdf2dh_helpers.h"

#include "testModels.h"

bool checkURDF2DHForAGivenChain(const KDL::CoDyCo::UndirectedTree& undirectedTree,
                                const std::string& base_frame,
                                const std::string& end_effector_frame)
{

    KDL::Chain kdl_chain;
    iCub::iKin::iKinLimb ikin_limb;

    //cerr << "urdf2dh test testing chain extraction between " << base_frame
    //     << " and " << end_effector_frame << endl;
    KDL::Tree kdl_rotated_tree = undirectedTree.getTree(base_frame);

    bool result = kdl_rotated_tree.getChain(base_frame,end_effector_frame,kdl_chain);
    if( !result )
    {
        cerr << "urdf2dh: Impossible to find " << base_frame << " or "
                << end_effector_frame << " in the URDF."  << endl;
            return false;
    }

    //
    // Convert the chain and the limits to an iKin chain (i.e. DH parameters)
    //
    KDL::JntArray qmin(kdl_chain.getNrOfJoints()),qmax(kdl_chain.getNrOfJoints());

    for(size_t i=0; i < qmin.rows(); i++ ) { qmin(i) = -10.0; qmax(i) = 10.0; }

    result = iKinLimbFromKDLChain(kdl_chain,ikin_limb,qmin,qmax,1000);
    if( !result )
    {
        cerr << "urdf2dh: Could not export KDL::Tree to iKinChain" << endl;
        return false;
    }

    return checkChainsAreEqual(kdl_chain,ikin_limb);
}

bool checkURDF2DH(std::string urdf_file_name)
{
    std::cerr << "Running checkURDF2DH for model " << urdf_file_name << std::endl;


    KDL::Tree kdl_tree;
    //
    // URDF --> KDL::Tree
    //
    bool root_inertia_workaround = true;
    if( !treeFromUrdfFile(urdf_file_name,kdl_tree,root_inertia_workaround) )
    {
        cerr << "urdf2dh_tests: Could not parse urdf robot model" << endl;
        std::cerr << "urdf2dh_tests: Please open an issue at https://github.com/robotology/idyntree/issues " << std::endl;

        return false;
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

    for(size_t base_frame_index = 0;
        base_frame_index < undirected_tree.getNrOfLinks();
        base_frame_index++ )
    {
        for(size_t end_effector_frame_index = 0;
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

          cerr << "urdf2dh_test: Checking " << base_frame << " <--> " << end_effector_frame << " transform." << std::endl;
          bool conversionOk = checkURDF2DHForAGivenChain(undirected_tree,base_frame,end_effector_frame);
          if( !conversionOk )
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
    for(size_t i=0; i < undirected_tree.getNrOfLinks(); i++ )
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

int main(int argc, char** argv)
{
    // If we specify an argument we run the test on a specific model
    bool test_success = true;
    if( argc > 2 )
    {
        std::string urdf_file_name  = argv[1];
        test_success = checkURDF2DH(urdf_file_name);
    }
    else
    {
        // Otherwise run the test on all the models
        for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
        {
            std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
            bool ok = checkURDF2DH(urdfFileName);
            test_success = test_success && ok;
        }
    }

    if( test_success )
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}


