/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
  
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>

#include <cfloat>


namespace KDL {
namespace CoDyCo {
namespace Regressors {   
    
double sparsity_index(const Eigen::MatrixXd & mat, const double tol)
    {
        int zero_elements = 0;
        for(int i=0; i < mat.rows(); i++ ) {
            for(int j=0; j < mat.cols(); j++ ) {
                if( fabs(mat(i,j)) < tol ) {
                    zero_elements++;
                }
            }
        }
        return ((double)zero_elements)/(mat.rows()*mat.cols());
    }
    
Eigen::MatrixXd zeroToZero(const Eigen::MatrixXd & input_mat, double tol)
{
    Eigen::MatrixXd output_mat;
    output_mat = input_mat;
    
    for(int i=0; i < output_mat.rows(); i++ ) {
        for(int j=0; j < output_mat.cols(); j++ ) {
            if( fabs(output_mat(i,j)) < tol ) {
                output_mat(i,j) = 0.0;
            }
        }
    }
    return output_mat;
}

int getKernelSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & kernel_basis_matrix, double tol, bool verbose)
{
    if( input_matrix.rows() == 0 ) {
        kernel_basis_matrix = Eigen::MatrixXd::Identity(input_matrix.cols(),input_matrix.cols());
        return 0;
    }
        //std::cout << "Called getRowSpaceBasis with input_matrix: " << std::endl;
        //std::cout << input_matrix << std::endl;
        //Probably can be improved using a different decomposition (QR?)
        //NOT REAL TIME!!
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeFullV);
        

        
        int n = input_matrix.rows();
        int m = input_matrix.cols();
        
        Eigen::VectorXd sigma = svd.singularValues();
        
        if(verbose) {  std::cout << "Singular values " << std::endl; std::cout << sigma << std::endl; }
        
        if( tol <= 0 ) {
            /** \todo find a better and consistend heuristic */
            //To avoid problem on numerically zero matrices
            if( sigma[0] >= sqrt(DBL_EPSILON) ) { 
                tol = 1000*sigma[0]*std::max(n,m)*DBL_EPSILON;
            } else {
                //Matrix is probably numerically zero
                //It is wise to consider all the matrix as a zero matrix
                tol = sqrt(DBL_EPSILON);
            }
        }
        
        int ll;
        for(ll=0; ll < sigma.size(); ll++ ) {
            if( sigma[ll] < tol ) { break;}
        }
      
        int rank = ll;
         
        Eigen::MatrixXd V = svd.matrixV();
        assert(V.cols() == V.rows());
        if( V.cols() != V.rows() ) { std::cout << "V is not square" << std::endl; }
        assert(rank <= m);
        
        kernel_basis_matrix.resize(m,m-rank);
        std::cout << "Matrix has rank " << rank << std::endl; std::cout << " m " << m << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl; 
        kernel_basis_matrix = V.block(0,rank,m,m-rank);
        std::cout << "basis calculated, tol used " << tol << std::endl; 
        
        return 0;
}

int getSubSpaceIntersection(const Eigen::MatrixXd & first_subspace, const Eigen::MatrixXd & second_subspace, Eigen::MatrixXd & result, double tol, bool verbose)
{
    std::cout << "getSubSpaceIntersection" << std::endl;
    
    std::cout << "first_subspace " << first_subspace.rows() << " " << first_subspace.cols() << std::endl;
    //std::cout << first_subspace << std::endl;
    std::cout << "second_subspace " << second_subspace.rows() << " " << second_subspace.cols() << std::endl;
    //std::cout << second_subspace << std::endl;
    
    if( tol <= 0.0 ) { tol = 1e-7; }
    //The input matrices columns in input should form a basis of a subspace of a common vector space
    if( first_subspace.rows() != second_subspace.rows() ) { return -1; }
    if( first_subspace.cols() > first_subspace.rows() ) { return -1; }
    if( second_subspace.cols() > second_subspace.rows() ) { return -1; }
    if( first_subspace.cols() == 0 || second_subspace.cols() == 0 ) { result.resize(first_subspace.rows(),0); return 0; }  
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(first_subspace.transpose()*second_subspace, Eigen::ComputeFullU | Eigen::ComputeThinV);
    
    Eigen::VectorXd sigma = svd.singularValues();
    //std::cout << "Sigma " << std::endl << sigma << std::endl;
    int intersection_size = 0;
    for(int i=0; i < sigma.size(); i++ ) {
        if( fabs(sigma[i]-1) < tol ) {
            intersection_size++;
        } else {
            break;
        }
    }
    
    std::cout << "intersection_size " << intersection_size << std::endl;

    
    if( intersection_size == 0 ) { result.resize(first_subspace.rows(),0); return 0; }  

    //result.resize(first_subspace.rows(),intersection_size);
    //std::cout << "Before compute" << std::endl;
    //std::cout << "After compute" << std::endl;
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();
    //std::cout << "After assignment" << std::endl;
    //std::cout << "Second subspace size: " << second_subspace.rows() << " " << second_subspace.cols() << std::endl;
    //std::cout << "V size: " << V.rows() << " " << V.cols() << std::endl;
    //std::cout << "intersection_size " << intersection_size << std::endl;
    
    for(int i=0; i < intersection_size; i++ ) {
        //result = (second_subspace*V.transpose()).block(0,0,second_subspace.rows(),intersection_size);
        result = (first_subspace*U.transpose().block(0,0,U.cols(),intersection_size));
        
    }

    return 0;
}

int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol, bool verbose)
{
    Eigen::VectorXd dummy;
    return getRowSpaceBasis(input_matrix,row_space_basis_matrix,tol,verbose,dummy);
}
    
int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol, bool verbose, Eigen::VectorXd & sigma)
{
    if( input_matrix.rows() == 0 ) {
        row_space_basis_matrix.resize(input_matrix.cols(),0);
        return 0;
    }
        std::cout << "Called getRowSpaceBasis " << std::endl;
        //std::cout << input_matrix << std::endl;
        //Probably can be improved using a different decomposition (QR?)
        //NOT REAL TIME!!
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeFullV);
        

        
        int n = input_matrix.rows();
        int m = input_matrix.cols();
        
        sigma = svd.singularValues();
        
        
        if( tol <= 0 ) {
            /** \todo find a better and consistend heuristic */
            //To avoid problem on numerically zero matrices
            if( sigma[0] >= sqrt(DBL_EPSILON) ) { 
                tol = 1000*sigma[0]*std::max(n,m)*DBL_EPSILON;
            } else {
                //Matrix is probably numerically zero
                //It is wise to consider all the matrix as a zero matrix
                tol = sqrt(DBL_EPSILON);
            }
        }
        
        int ll;
        for(ll=0; ll < sigma.size(); ll++ ) {
            if( sigma[ll] < tol ) { break;}
        }
      
        int rank = ll;
         
        Eigen::MatrixXd V = svd.matrixV();
        assert(V.cols() == V.rows());
        if( V.cols() != V.rows() ) { std::cout << "V is not square" << std::endl; }
        assert(rank <= m);
        
        row_space_basis_matrix.resize(m,rank);
        std::cout << "Matrix has rank " << rank << std::endl; std::cout << " m " << m << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl; 
        row_space_basis_matrix = V.block(0,0,m,rank);
        std::cout << "basis calculated, tol used " << tol << std::endl; 
        
        return 0;
}

}

}

}