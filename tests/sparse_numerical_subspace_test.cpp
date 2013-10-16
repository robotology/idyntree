#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

#include <dirl/dynamicRegressorGenerator.hpp>

#include <kdl_codyco/regressor_loops.hpp>

#include <kdl_codyco/regressor_utils.hpp>

#include "test_models.hpp"

#include <ctime>

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace dirl;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int main()
{    
    srand(time(NULL));
    
    Tree test_tree = TestSingleJoint();
    
    
    //Then create the regressor generator 
    DynamicRegressorGenerator regressor(test_tree);
    
    regressor.addBaseRegressorRows();
    regressor.addAllTorqueRegressorRows();

    
    Eigen::MatrixXd base_space, sparse_base_space;
    
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeNumericalIdentifiableSubspace(base_space)~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    int ret = regressor.computeNumericalIdentifiableSubspace(base_space);
    
    if( ret != 0 ) { std::cerr << "computeNumericalIdentifiableSubspace failed with error code " << ret << std::endl; return ret; }
   
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspace(base_space)~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

    ret = regressor.computeSparseNumericalIdentifiableSubspace(sparse_base_space);
    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspace failed with error code " << ret << std::endl; return ret; }

    
    std::cout << "Basis of the base subspace calculated in the classical way" << std::endl;
    std::cout << base_space << std::endl;
    
    std::cout << "Basis of the base subspace calculated with the sparse algorithm" << std::endl;
    std::cout << sparse_base_space << std::endl;
    
    if( sparse_base_space.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different (normal: " << base_space.cols() << " sparse: " << sparse_base_space.cols() << " ) " << std::endl; return -1; } 
    
    int np = sparse_base_space.rows();
    int nbp = sparse_base_space.cols();
    
    Eigen::MatrixXd test_residual = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space;
    
    std::cout << "Norm of the residual matrix " << test_residual.norm() << std::endl;
    
    if( test_residual.norm() > 1e-10 ) { return -1; }
    
    return 0;
}
