#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/undirectedtree.hpp>

#include "kdl_codyco/rnea_loops.hpp"
#include <kdl_codyco/regressor_utils.hpp>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_codyco/treedynparam.hpp>
#include <kdl_codyco/floatingjntspaceinertiamatrix.hpp>


#include <kdl_codyco/rnea_loops.hpp>


#include "test_models.hpp"

#include <ctime>

using namespace KDL;
using namespace KDL::CoDyCo;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int main()
{    
    JntArray q, dq, ddq, torques_slv;
    Wrenches f_slv,f_ext;
    Wrench base_force_slv;
    Twist base_vel, base_acc;   
    
    JntArray torques_loop_one;
    Wrenches f_loop_one;
    Wrench base_force_loop_one;
    std::vector<Twist> v_loop_one,a_loop_one;
    
    JntArray torques_loop_two;
    Wrenches f_loop_two, f_gi_loop_two;
    Wrench base_force_loop_two(Vector(1,2,3),Vector(4,5,6));
    std::vector<Twist> v_loop_two,a_loop_two;
    
    srand(time(NULL));
    
    Tree test_tree = TestHumanoid();
    UndirectedTree test_undirected_tree(test_tree);
    Traversal test_traversal;
    
    test_undirected_tree.compute_traversal(test_traversal);
    
    //Creating several solvers: 
    //one for proper floating base inverse dynamics
    TreeIdSolver_RNE rne_idsolver(test_tree);
    
    //one by manually manipulating loops
    
    //Create the variables
    q= dq = ddq = torques_slv = torques_loop_one = torques_loop_two = JntArray(test_tree.getNrOfJoints());
    f_ext = f_slv = f_loop_one = f_loop_two = f_gi_loop_two = std::vector<Wrench>(test_tree.getNrOfSegments(),KDL::Wrench::Zero());
    v_loop_one = a_loop_one = v_loop_two = a_loop_two = std::vector<Twist>(test_undirected_tree.getNrOfLinks(),KDL::Twist::Zero());
    
    
    for(int i=0; i < (int)test_tree.getNrOfJoints(); i++ )
    {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }
    
    base_vel = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    base_acc = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));

    //Inserting the random input data in both solvers, while checking all went well
    if( rne_idsolver.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques_slv,base_force_slv) != 0 ) return -1;
    
    //Tryng the same with the normal loops
    bool ret = true;
    ret = ret && rneaKinematicLoop(test_undirected_tree,q,dq,ddq,test_traversal,base_vel,base_acc,v_loop_one,a_loop_one) == 0;
    ret = ret && rneaDynamicLoop(test_undirected_tree,q,test_traversal,v_loop_one,a_loop_one,f_ext,f_loop_one,torques_loop_one,base_force_loop_one) == 0;

    //Tryng the same with the modified loops
    ret = ret && rneaKinematicLoop(test_undirected_tree,q,dq,ddq,test_traversal,base_vel,base_acc,v_loop_two,a_loop_two,f_gi_loop_two) == 0;
    ret = ret && rneaDynamicLoop(test_undirected_tree,q,test_traversal,f_gi_loop_two,f_ext,f_loop_two,torques_loop_two,base_force_loop_two) == 0;

    assert(ret);
    
    //Build generalized_torques
    Eigen::VectorXd generalized_tau_slv = toEigen(base_force_slv,torques_slv);
    Eigen::VectorXd generalized_tau_loop_one = toEigen(base_force_loop_one,torques_loop_one);
    Eigen::VectorXd generalized_tau_loop_two = toEigen(base_force_loop_two,torques_loop_two);

    
    std::cout << "Generalized Torques obtained with RNE" << std::endl << generalized_tau_slv << std::endl;
    std::cout << "Generalized Torques obtained with original loop" << std::endl << generalized_tau_loop_one << std::endl;
    std::cout << "Generalized Torques obtained with modified loop" << std::endl << generalized_tau_loop_two << std::endl;

    std::cout << "Generalized Torques obtained with RNE - original loop" << std::endl << generalized_tau_slv-generalized_tau_loop_one << std::endl;
    std::cout << "Generalized Torques obtained with RNE - modified loop" << std::endl << generalized_tau_slv-generalized_tau_loop_two << std::endl;
    
    for(int i=0; i < (int)test_traversal.getNrOfVisitedLinks(); i++) {
            LinkMap::const_iterator link_it = test_traversal.getOrderedLink(i);
            int link_nmbr = link_it->getLinkIndex();
            std::cout << "Link " << link_nmbr << std::endl;
            std::cout << "F_ext " << f_ext[link_nmbr] << std::endl;
            std::cout << "f_loop_one " << f_loop_one[link_nmbr] << std::endl;
            std::cout << "f_loop_two " << f_loop_two[link_nmbr] << std::endl;
            std::cout << "f_loop_one-f_loop_two " << f_loop_one[link_nmbr]-f_loop_two[link_nmbr] << std::endl;
            std::cout << "f_gi_loop_two " << f_gi_loop_two[link_nmbr] << std::endl;
    }    
    if( (generalized_tau_slv-generalized_tau_loop_one).norm() > 1e-10 )  return -1;
    if( (generalized_tau_slv-generalized_tau_loop_two).norm() > 1e-10 )  return -1;

    return 0;
}
