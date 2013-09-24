#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/treegraph.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

#include <drrl/dynamicRegressorGenerator.hpp>

#include <kdl_codyco/regressor_loops.hpp>

#include <kdl_codyco/regressor_utils.hpp>



#include "test_models.hpp"

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace DRRL;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int main()
{    
    Tree test_tree = TestHumanoid();
    
    //Creating relative TreeGraph
    TreeGraph tree_graph(test_tree);
    
    JntArray q,dq,ddq,torques;
    KDL::Twist base_vel, base_acc;
    Wrench base_wrench;
    std::vector<Twist> v,a;
    std::vector<Wrench> f,f_ext;
    std::vector<Frame> X_dynamic_base(tree_graph.getNrOfLinks());
    
    //arbitrary traversal
    std::string kinematic_base = "lleg_seg4";
    Traversal kinematic_traversal;
    tree_graph.compute_traversal(kinematic_traversal,kinematic_base);
    Traversal dynamic_traversal;
    
    //It is necessary to use a dynamic base that is not in the considered subtree for the regressors
    tree_graph.compute_traversal(dynamic_traversal,"rleg_seg8");
    //tree_graph.compute_traversal(dynamic_traversal);
    q = dq = ddq = torques = JntArray(tree_graph.getNrOfDOFs());
    v = a = std::vector<Twist>(tree_graph.getNrOfLinks());
    f = f_ext = std::vector<Wrench>(tree_graph.getNrOfLinks(),KDL::Wrench::Zero());
    
    for(int i=0; i < tree_graph.getNrOfDOFs(); i++ )
    {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }
    
    base_vel = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    base_acc = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));

    
    //Calculating the velocity and acceleration for each link
    rneaKinematicLoop(tree_graph,q,dq,ddq,kinematic_traversal,base_vel,base_acc,v,a);
    rneaDynamicLoop(tree_graph,q,dynamic_traversal,v,a,f_ext,f,torques,base_wrench);
    
    //Get the frame between each link and the base
    getFramesLoop(tree_graph,q,dynamic_traversal,X_dynamic_base);
   
    //Create ft_list
    std::vector<std::string> ft_names;
    ft_names.push_back("lleg_ft0");
    ft_names.push_back("lleg_ft1");
    ft_names.push_back("rleg_ft0");
    ft_names.push_back("rleg_ft1");
    ft_names.push_back("larm_ft0");
    ft_names.push_back("larm_ft1");
    ft_names.push_back("rarm_ft0");
    ft_names.push_back("rarm_ft1");
    ft_names.push_back("head_ft0");
    
    //Create fake links
    std::vector<std::string> fake_links;
    fake_links.push_back("larm_seg4");
    fake_links.push_back("rarm_seg4");
    fake_links.push_back("lleg_seg4");
    fake_links.push_back("rleg_seg4");
    
#ifndef NDEBUG
    std::cout << "There are " << tree_graph.getNrOfLinks() << " links , " << tree_graph.getNrOfLinks()-fake_links.size() << " real and " << fake_links.size() << " fake " << std::endl;
#endif
    
    FTSensorList ft_list(tree_graph,ft_names);
    
    //Generate random offset data
    std::vector<Wrench> measured_wrenches_offset(ft_names.size());
    for( int i=0; i < ft_names.size(); i++ ) {
        measured_wrenches_offset[i] =  Wrench(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    }
    
    //Get measured wrenches from RNEA
    std::vector<Wrench> measured_wrenches(ft_names.size());
    for( int i=0; i < ft_names.size(); i++ ) {
        measured_wrenches[i] = ft_list.estimateSensorWrenchFromRNEA(i,dynamic_traversal,f) + measured_wrenches_offset[i];
    }
      
    //Then create the regressor generator 
    DynamicRegressorGenerator regressor(test_tree,kinematic_base,ft_names,true,fake_links);
    
    //Adding some subtrees
    std::vector< std::string > subtree_torso;
    std::vector< std::string > subtree_bigger_torso;
    std::vector< std::string > subtree_torso_nohead;
    std::vector< std::string > subtree_rhand;
    std::vector< std::string > subtree_middle_lleg;
    std::vector< std::string > subtree_head;
    
    subtree_torso.push_back("rarm_seg1");
    subtree_torso.push_back("larm_seg1");
    subtree_torso.push_back("rleg_seg1");
    subtree_torso.push_back("lleg_seg1");
    subtree_torso.push_back("head_seg2");
    
    subtree_bigger_torso.push_back("rarm_seg5");
    subtree_bigger_torso.push_back("larm_seg5");
    subtree_bigger_torso.push_back("rleg_seg5");
    subtree_bigger_torso.push_back("lleg_seg5");
    subtree_bigger_torso.push_back("head_seg2");
    
    subtree_torso_nohead.push_back("rarm_seg1");
    subtree_torso_nohead.push_back("larm_seg1");
    subtree_torso_nohead.push_back("rleg_seg1");
    subtree_torso_nohead.push_back("lleg_seg1");
    
    subtree_rhand.push_back("rarm_seg6");
    
    subtree_middle_lleg.push_back("lleg_seg2");
    subtree_middle_lleg.push_back("lleg_seg5");
    
    subtree_head.push_back("head_seg3");
    
    int ret_val = 0;
    
    ret_val = regressor.addSubtreeRegressorRows(subtree_torso);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of torso " << std::endl; return -1; }
    ret_val = regressor.addSubtreeRegressorRows(subtree_bigger_torso);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of bigger_torso " << std::endl; return -1; }
    ret_val = regressor.addSubtreeRegressorRows(subtree_torso_nohead);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of torso_nohead " << std::endl; return -1; }
    ret_val = regressor.addSubtreeRegressorRows(subtree_rhand);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of hand " << std::endl; return -1; }
    ret_val = regressor.addSubtreeRegressorRows(subtree_middle_lleg);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of thigh " << std::endl; return -1; }
    ret_val = regressor.addSubtreeRegressorRows(subtree_head);
    if( ret_val != 0 ) { std::cerr << "Problem in adding regressor of head  " << std::endl; return -1; }

    
    
    regressor.setRobotState(q,dq,ddq,base_vel,base_acc);

    for( int i=0; i < ft_names.size(); i++ ) {
        regressor.setFTSensorMeasurement(i,measured_wrenches[i]);
    }
    
    Eigen::MatrixXd regr(regressor.getNrOfOutputs(),regressor.getNrOfParameters());
    Eigen::VectorXd kt(regressor.getNrOfOutputs());
    
    regressor.computeRegressor(regr,kt);
    
    #ifndef NDEBUG
    std::cout << "Tree graph nrOfLinks" << tree_graph.getNrOfLinks() << std::endl;
    std::cout << "Regressor nrOfParam: " << regressor.getNrOfParameters() << " nrOfOutputs " << regressor.getNrOfOutputs() << std::endl;
    std::cout << regressor.getDescriptionOfParameters() << std::endl;
    #endif
    
    
    //if the ft offset where not considered, then the parameter are only the inertial parameter of the tree
    Eigen::VectorXd parameters(regressor.getNrOfParameters());
    parameters.setZero();
    
    inertialParametersVectorLoopFakeLinks(tree_graph,parameters,fake_links);
    //Adding fake ft offsets
    int NrOfRealLinks = 10*(tree_graph.getNrOfLinks()-fake_links.size());
    for( int ft_id =0; ft_id < ft_names.size(); ft_id++ ) {
      for( int www=0; www < 6; www++ ) {
        parameters[NrOfRealLinks+6*ft_id+www] = measured_wrenches_offset[ft_id](www);
      }
    }
    
    
    Eigen::VectorXd kt_obtained = regr*parameters;
        
  
    
    std::cout << "kt" << std::endl;
    std::cout << (kt) << std::endl;
    
    
    std::cout << "kt_obtained" << std::endl;
    std::cout << (kt_obtained) << std::endl;
    
    std::cout << "kt-kt_obtained" << std::endl;
    std::cout << (kt-kt_obtained) << std::endl;
    
    
    if( (kt-kt_obtained).norm() > 1e-10 )  return -1;
    
    return 0;
}
