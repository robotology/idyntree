/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <chrono>

#include <kdl_codyco/regressor_loops.hpp>

#include <kdl_codyco/regressor_utils.hpp>

#include "test_models.hpp"

#include <ctime>

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int main()
{
    srand(time(NULL));

    //Tree test_tree = TestSingleJoint();
    Tree test_tree = TestSimpleHumanoid();


    //Then create the regressor generator
    DynamicRegressorGenerator regressor(test_tree);

    regressor.addBaseRegressorRows();
    regressor.addAllTorqueRegressorRows();


    Eigen::MatrixXd base_space, sparse_base_space, sparse_base_space_simple, sparse_base_space_adv_algo, sparse_base_space_simple_golub, sparse_base_space_simple_algo;
    //profiling
    std::chrono::time_point<std::chrono::system_clock> start, end;


    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeNumericalIdentifiableSubspace(base_space)~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    //int ret=0;
    start = std::chrono::system_clock::now();
    int ret = regressor.computeNumericalIdentifiableSubspace(base_space);
    end = std::chrono::system_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if( ret != 0 ) { std::cerr << "computeNumericalIdentifiableSubspace failed with error code " << ret << std::endl; return ret; }


    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspaceSimplePaper~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    start = std::chrono::system_clock::now();

    ret = regressor.computeSparseNumericalIdentifiableSubspaceSimplePaper(sparse_base_space_simple);
    end = std::chrono::system_clock::now();

    auto elapsed_simple_paper = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspaceSimplePaper failed with error code " << ret << std::endl; return ret; }


    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspaceAdvancedPaper~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    start = std::chrono::system_clock::now();
    ret = regressor.computeSparseNumericalIdentifiableSubspaceAdvancedPaper(sparse_base_space);
    end = std::chrono::system_clock::now();

    auto elapsed_adv_paper = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspaceAdvancedPaper failed with error code " << ret << std::endl; return ret; }

      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspaceSimpleGolub~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    start = std::chrono::system_clock::now();

    ret = regressor.computeSparseNumericalIdentifiableSubspaceSimpleGolub(sparse_base_space_simple_golub);
    end = std::chrono::system_clock::now();

    auto elapsed_simple_golub = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspaceSimpleGolub failed with error code " << ret << std::endl; return ret; }


    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspaceSimpleAlgorithm~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    start = std::chrono::system_clock::now();
    ret = regressor.computeSparseNumericalIdentifiableSubspaceSimpleAlgorithm(sparse_base_space_simple_algo);
    end = std::chrono::system_clock::now();

    auto elapsed_simple_algo = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspaceSimpleAlgorithm failed with error code " << ret << std::endl; return ret; }


    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~computeSparseNumericalIdentifiableSubspaceAdvancedAlgorithm~~~~~~~~~" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

    start = std::chrono::system_clock::now();
    ret = regressor.computeSparseNumericalIdentifiableSubspaceAdvancedAlgorithm(sparse_base_space_adv_algo);
    end = std::chrono::system_clock::now();

    auto elapsed_adv_algo = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if( ret != 0 ) { std::cerr << "computeSparseNumericalIdentifiableSubspaceAdvancedAlgorithm failed with error code " << ret << std::endl; return ret; }

    int np = regressor.getNrOfParameters();

    Eigen::IOFormat simple_frmt(2);


    /*
    std::cout << "Basis of the base subspace calculated in the classical way" << std::endl;
    std::cout << zeroToZero(base_space.transpose()).format(simple_frmt) << std::endl;

    std::cout << "Basis of the base subspace calculated with the sparse algorithm" << std::endl;
    std::cout << zeroToZero(sparse_base_space.transpose()).format(simple_frmt) << std::endl;


    std::cout << "Basis of the base subspace calculated with the sparse algorithm v4" << std::endl;
    std::cout << zeroToZero(sparse_base_space_v4.transpose()).format(simple_frmt) << std::endl;


    std::cout << "Basis of the base subspace calculated with the sparse algorithm v3" << std::endl;
    std::cout << zeroToZero(sparse_base_space_v3.transpose()).format(simple_frmt) << std::endl;
    */

    if( sparse_base_space.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different (normal: " << base_space.cols() << " sparse (advanced paper): " << sparse_base_space.cols() << " ) " << std::endl; return -1; }
    if( sparse_base_space_simple.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different (normal: " << base_space.cols() << " sparse (simple paper): " << sparse_base_space_simple.cols() << " ) " << std::endl; return -1; }
    if( sparse_base_space_adv_algo.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different advanced algorithm (normal: " << base_space.cols() << " sparse (advanced algorithm): " << sparse_base_space_adv_algo.cols() << " ) " << std::endl; return -1; }
    if( sparse_base_space_simple_algo.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different simple algorithm (normal: " << base_space.cols() << " sparse (simple algorithm): " << sparse_base_space_simple_algo.cols() << " ) " << std::endl; return -1; }
    if( sparse_base_space_simple_golub.cols() != base_space.cols() ) { std::cerr << "Error the number of numerical parameters is different simple algorithm (normal: " << base_space.cols() << " sparse (simple algorithm): " << sparse_base_space_simple_algo.cols() << " ) " << std::endl; return -1; }


    np = sparse_base_space.rows();
    int nbp = sparse_base_space.cols();

    Eigen::MatrixXd test_residual = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space;
    Eigen::MatrixXd test_residual_adv_algo = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space_adv_algo;
    Eigen::MatrixXd test_residual_simple = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space_simple;
    Eigen::MatrixXd test_residual_simple_algo = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space_simple_algo;
    Eigen::MatrixXd test_residual_simple_golub = (Eigen::MatrixXd::Identity(np,np)-base_space*base_space.transpose())*sparse_base_space_simple_golub;

    double tol = 1e-10;

    int mat_elems = base_space.rows()*base_space.cols();
    std::cout << "Sparsity of original base " << sparsity_index(base_space,tol) << " ( " << sparsity_index(base_space,tol)*mat_elems << " ) " << std::endl;
    std::cout << "Sparsity of sparse base (simple paper) " << sparsity_index(sparse_base_space_simple,tol) << " ( " << sparsity_index(sparse_base_space_simple,tol)*mat_elems << " ) " << std::endl;
    std::cout << "Sparsity of sparse base (advanced paper) " << sparsity_index(sparse_base_space,tol) << " ( " << sparsity_index(sparse_base_space,tol)*mat_elems << " ) " << std::endl;
    std::cout << "Sparsity of sparse base (simple_algorithm) " << sparsity_index(sparse_base_space_simple_algo,tol) << " ( " << sparsity_index(sparse_base_space_simple_algo,tol)*mat_elems << " ) " << std::endl;
    std::cout << "Sparsity of sparse base (advanced algorithm) " << sparsity_index(sparse_base_space_adv_algo,tol) << " ( " << sparsity_index(sparse_base_space_adv_algo,tol)*mat_elems << " ) " << std::endl;
    std::cout << "Sparsity of sparse base (simple golub algorithm) " << sparsity_index(sparse_base_space_simple_golub,tol) << " ( " << sparsity_index(sparse_base_space_simple_golub,tol)*mat_elems << " ) " << std::endl;


    std::cout << "Norm of the residual matrix simple paper " << test_residual_simple.norm() << std::endl;
    std::cout << "Norm of the residual matrix advanced paper " << test_residual.norm() << std::endl;
    std::cout << "Norm of the residual matrix simple algorithm " << test_residual_simple_algo.norm() << std::endl;
    std::cout << "Norm of the residual matrix advanced algorithm " << test_residual_adv_algo.norm() << std::endl;
    std::cout << "Norm of the residual matrix simple golub " << test_residual_simple_golub.norm() << std::endl;


    /*
    std::cout << "Report original base" << std::endl;
    std::cout << regressor.analyseBaseSubspace(base_space,tol);
    std::cout << "Report sparse base  " << std::endl;
    std::cout << regressor.analyseBaseSubspace(sparse_base_space,tol);
    std::cout << "Report sparse base v4" << std::endl;
    std::cout << regressor.analyseBaseSubspace(sparse_base_space_v4,tol);
    std::cout << "Report sparse base v3" << std::endl;
    std::cout << regressor.analyseBaseSubspace(sparse_base_space_v3,tol);
    */

    std::cout << "Report original base" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(base_space,tol,true);
    std::cout << "Report sparse base (simple paper)" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(sparse_base_space_simple,tol);
    std::cout << "Report sparse base (advanced paper)" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(sparse_base_space,tol);
    std::cout << "Report sparse base Simple Algorithm" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(sparse_base_space_simple_algo,tol);
    std::cout << "Report sparse base Advanced Algorithm" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(sparse_base_space_adv_algo,tol);
    std::cout << "Report sparse base Simple golub" << std::endl;
    std::cout << regressor.analyseSparseBaseSubspace(sparse_base_space_simple_golub,tol);


    std::cout << "Computation time classical algorithm" << std::endl;
    std::cout << elapsed.count() << std::endl;
    std::cout << "Computation time simple paper" << std::endl;
    std::cout << elapsed_simple_paper.count() << std::endl;
    std::cout << "Computation time advanced paper" << std::endl;
    std::cout << elapsed_adv_paper.count() << std::endl;
    std::cout << "Computation time simple algorithm" << std::endl;
    std::cout << elapsed_simple_algo.count() << std::endl;
    std::cout << "Computation time advanced algorithm" << std::endl;
    std::cout << elapsed_adv_algo.count() << std::endl;
    std::cout << "Computation time simple golub" << std::endl;
    std::cout << elapsed_simple_golub.count() << std::endl;



    if( test_residual.norm() > tol) { return -1; }
    if( test_residual_simple.norm() > tol) { return -1; }
    if( test_residual_adv_algo.norm() > tol) { return -1; }
    if( test_residual_simple_algo.norm() > tol) { return -1; }
    if( test_residual_simple_golub.norm() > tol) { return -1; }


    return 0;
}
