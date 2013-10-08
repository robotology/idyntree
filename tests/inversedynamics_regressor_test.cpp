#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_codyco/treeinertialparameters.hpp>

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

    srand(time(NULL));
    
    Tree test_tree = TestHumanoid();
    
        Eigen::MatrixXd regressor;
    
    regressor.resize(6+test_tree.getNrOfJoints(),10*test_tree.getNrOfSegments());
    
    //Creating several solvers: 
    //one for floating base inverse dynamics
    TreeIdSolver_RNE rne_idsolver(test_tree);
    
    //one for floating base inverse dynamics expressed as a regressor of inertial parameters
    TreeInertialParametersRegressor regressor_idsolver(test_tree);
    
    JntArray q,dq,ddq,torques;
    Wrenches f,f_ext;
    Wrench base_force;
    Twist base_vel, base_acc;   

    
    q = dq = ddq = torques = JntArray(test_tree.getNrOfJoints());
    f = f_ext = std::vector<Wrench>(test_tree.getNrOfSegments(),KDL::Wrench::Zero());
    
    for(int i=0; i < test_tree.getNrOfJoints(); i++ )
    {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }
    
    base_vel = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
    base_acc = Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));

    //Inserting the random input data in both solvers, while checking all went well
    if( rne_idsolver.CartToJnt(q,dq,ddq,base_vel,base_acc,f_ext,torques,base_force) != 0 ) return -1;
    if( regressor_idsolver.dynamicsRegressor(q,dq,ddq,base_vel,base_acc,regressor) != 0 ) return -1;
    
    Eigen::VectorXd generalized_tau_rne = toEigen(base_force,torques);
    Eigen::VectorXd generalized_tau_regr = regressor*regressor_idsolver.getInertialParameters();
    
    //std::cout << "Generalized Torques obtained with RNE" << std::endl << generalized_tau_rne << std::endl;
    //std::cout << "Generalized Torques obtained with Regressor" << std::endl << generalized_tau_regr << std::endl;
    //std::cout << "Difference between the two" << std::endl << generalized_tau_rne-generalized_tau_regr << std::endl;

    
    if( (generalized_tau_regr-generalized_tau_rne).norm() > 1e-10 )  return -1;
    
    return 0;
}
