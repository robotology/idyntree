#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/regressors/dataset/DynamicDatasetFile.hpp>
#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <cstdlib>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>
#include <fstream>


using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;

#define LEFT_ELBOW_TORQUE_INDEX 2
#define RIGHT_ELBOW_TORQUE_INDEX 8

#define LEFT_ARM_FT_INDEX 0
#define RIGHT_ARM_FT_INDEX 1


/**
 * Some helper function for converting between Yarp,KDL and Eigen
 * matrix types.
 *
 */
bool toYarp(const KDL::Wrench & ft, yarp::sig::Vector & ft_yrp)
{
    if( ft_yrp.size() != 6 ) { ft_yrp.resize(6); }
    for(int i=0; i < 6; i++ ) {
        ft_yrp[i] = ft[i];
    }
}

bool toKDL(const yarp::sig::Vector & ft_yrp, KDL::Wrench & ft)
{
    for(int i=0; i < 6; i++ ) {
        ft[i] = ft_yrp[i];
    }
}

bool toYarp(const Eigen::VectorXd & vec_eigen, yarp::sig::Vector & vec_yrp)
{
    if( vec_yrp.size() != vec_eigen.size() ) { vec_yrp.resize(vec_eigen.size()); }
    if( memcpy(vec_yrp.data(),vec_eigen.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}

bool toEigen(const yarp::sig::Vector & vec_yrp, Eigen::VectorXd & vec_eigen)
{
 if( vec_yrp.size() != vec_eigen.size() ) { vec_eigen.resize(vec_yrp.size()); }
    if( memcpy(vec_eigen.data(),vec_yrp.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}


int main(int argc, char ** argv)
{
    yarp::os::Property opt;

    opt.fromCommand(argc,argv);

    std::cout << "iCub_subtree_base_regressor_example: " << std::endl;


    //Here we define the subtrees, they are simply defined as the
    //name of the link attached to the ft sensor along where
    //we "cut" the three
    std::vector<std::string> l_arm_subtree, r_arm_subtree, arm_subtree;

    l_arm_subtree.push_back("l_arm");
    r_arm_subtree.push_back("r_arm");

    arm_subtree = r_arm_subtree;

    //We extract the iCub model from the old
    //iDyn library, but soon we will load it
    //directly from URDF


    //Defining some options for selecting between the
    //right arm and the left arm

    //Can be RIGHT_ARM_FT_INDEX or LEFT_ARM_FT_INDEX
    const int ft_dataset_sensor_index = RIGHT_ARM_FT_INDEX;

    //Can be l_upper_arm or r_upper_arm
    std::string dynamic_base = "r_upper_arm";

    std::string ft_sensor_name = "r_arm_ft_sensor";

    std::vector<std::string> ft_names;
    ft_names.push_back(ft_sensor_name);

    //We specify which link are actually frames
    std::vector<std::string> fake_link_names;
    fake_link_names.push_back("torso");
    fake_link_names.push_back("imu_frame");
    fake_link_names.push_back("l_gripper");
    fake_link_names.push_back("r_gripper");
    fake_link_names.push_back("l_sole");
    fake_link_names.push_back("r_sole");
    fake_link_names.push_back("l_wrist_1");
    fake_link_names.push_back("r_wrist_1");

    std::string root_link_name = "root_link";



    iCub::iDynTree::iCubTree_version_tag tag;
    //iCubParis02 is a v2 robot
    tag.head_version = 2;
    tag.legs_version = 2;

    iCub::iDynTree::iCubTree icub_tree_model(tag);
    KDL::Tree icub_kdl_tree = icub_tree_model.getKDLTree();
    KDL::CoDyCo::UndirectedTree icub_kdl_undirected_tree = icub_tree_model.getKDLUndirectedTree();
    KDL::CoDyCo::TreeSerialization icub_serialization =icub_tree_model.getKDLUndirectedTree().getSerialization();

    //Define the list of FT sensors in the model
    KDL::CoDyCo::FTSensorList icub_ft_list(icub_kdl_undirected_tree,ft_names);

    //Define the regressor generator: we pass several options
    //Some interesting ones are consider_ft_offset, that enables/disable
    //the use of offsets to the sensors as parameters in the model
    bool consider_ft_offset = true;
    bool verbose = false;
    DynamicRegressorGenerator ft_regressor_generator(icub_kdl_tree,
                                                  root_link_name,
                                                  ft_names,
                                                  consider_ft_offset,
                                                  fake_link_names,
                                                  icub_serialization,verbose);


    //Adding the subtree base regressor to the generator:
    //    (1) 6 for ft sensors (with offset)
    int ret = ft_regressor_generator.addSubtreeRegressorRows(arm_subtree);
    assert(ret == 0);

    //Create some bogus joint values (all to zero now)
    //please note that the serialization of this vector
    //(i.e. which value of the vector corresponds to which
    //joint) follows the convention specified in icub_serialization
    KDL::JntArray q, dq, ddq;
    yarp::sig::Vector q_yarp, dq_yarp, ddq_yarp;
    q.resize(ft_regressor_generator.getNrOfDOFs());
    SetToZero(q);
    q_yarp.resize(q.rows(),0.0);
    dq_yarp = q_yarp;
    ddq_yarp = q_yarp;
    dq = q;
    ddq = q;

    //The gravity is defined in the frame of the root link
    //in this case it is passed as the root_link_name variable
    //to the regressor generator
    const double g = 9.806;
    KDL::Twist gravity(KDL::Vector(0.0,0.0,g),KDL::Vector(0.0,0.0,0.0));

    //Creating the matrix containing the ft_regressor and the ft known terms
    // ft_regressor*parameters = ft_kt
    Eigen::MatrixXd ft_regressor(ft_regressor_generator.getNrOfOutputs(),ft_regressor_generator.getNrOfParameters());
    Eigen::VectorXd ft_kt(ft_regressor_generator.getNrOfOutputs());

    //Matrix whose columns are the orthonormal basis of the identifiable subspace
    //The theory is :
    // parameters = base_parameters_subspace*base_parameters
    // hence:
    // ft_regressor*base_parameters_subspace*base_parameters = ft_kt
    // ft_regressor_base*base_parameters = ft_kt
    // where ft_regressor base is
    // ft_regressor_base = ft_regressor*base_parameters_subspace
    Eigen::MatrixXd base_parameters_subspace;

    int ret_value = 0;
    ret_value = ft_regressor_generator.computeNumericalIdentifiableSubspace(base_parameters_subspace);
    assert( ret_value == 0 );

    int nr_of_base_parameters = base_parameters_subspace.cols();

    Eigen::MatrixXd ft_regressor_base(ft_regressor_generator.getNrOfOutputs(),nr_of_base_parameters);

    //From the regressor generator is also possible to get some human readable description of outputs
    // and parameters, useful for debugging
    std::cout << "Regressor outputs: " << std::endl << ft_regressor_generator.getDescriptionOfOutputs() << std::endl;
    std::cout << "Regressor parameters: " << std::endl << ft_regressor_generator.getDescriptionOfParameters() << std::endl;

    return 0;
}
