/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
  
#include <dirl/sparseNumericalBaseParameters.hpp>

#include <cmath>
#include <iostream>

using namespace KDL::CoDyCo;

namespace dirl
{

/*
int DynamicRegressorGenerator::computeForwardSparseNumericalIdentifiableSubspace(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    
    if( n_samples < 0 ) { if(verbose) { std::cerr << "Error: number of samples of calculating the identifiable subspace cannot be negative" << std::endl; } return -1;  }
    
   
    int no = getNrOfOutputs();
    int np = getNrOfParameters();
    int nj = NrOfDOFs;
    
    //Generated robot state
    KDL::JntArray q(nj),dq(nj),ddq(nj);
    KDL::Twist a,v;
    
    
    Eigen::MatrixXd A(np,np); //working matrix
    Eigen::MatrixXd regressor(no,np);
    Eigen::VectorXd kt(no);
    
    Eigen::MatrixXd V(np,np);
    Eigen::VectorXd sigma(np);
    
    Eigen::MatrixXd dense_basis, sparse_basis;
    
    //Take track of the junction that come before the considered link
    std::vector<bool> considered_junctions(.getNrOfJunctions());
    
    //Calculate sequentially the subspace base, adding a link at the time
    for(int i=0; i < .getNrOfLinks(); i++ ) {
        
        /////////////////////////
        /// Subspace till a given tree
        ////////////
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.order[i];
        
        #ifndef NDEBUG
        std::cerr << "Generating samples for DOFs before of link " << link_it->getName() << std::endl;
        #endif
        
        for(int sample=0; sample < n_samples; sample++ ) {
            //generated suitable state, exciting only the dofs before the current considered link
            
            //Initially set to zero all the coordinates, then generated random only for the dof before the considered link
            SetToZero(q);
            SetToZero(dq);
            SetToZero(ddq);
            
            
            //Enumerate all the DOFs and junctions before the considered link
            //By default none of them
            fill(considered_junctions.begin(),considered_junctions.end(),false);
            
            KDL::CoDyCo::LinkMap::const_iterator successor_it = link_it;
            KDL::CoDyCo::LinkMap::const_iterator predecessor_it = dynamic_traversal.parent[link_it->getLinkIndex()]; 
            
            while( predecessor_it != .getInvalidLinkIterator() ) {
                KDL::CoDyCo::JunctionMap::const_iterator joint_it = successor_it->getAdjacentJoint(predecessor_it);
                
                considered_junctions[joint_it->getJunctionIndex()] = true;
                
                assert( joint_it->getNrOfDOFs() <= 1);
                
                if( joint_it->getNrOfDOFs() == 1 ) {
                    #ifndef NDEBUG
                    if( sample == 0 ) { std::cerr << "Exciting dof " << joint_it->getName() << std::endl; }
                    #endif
                    int dof_index = joint_it->getDOFIndex();
                    q(dof_index) = random_double(); 
                    
                    if( !static_regressor ) {
                        dq(dof_index) = random_double();
                        ddq(dof_index) = random_double();
                    }
                }
                
                successor_it = predecessor_it;
                predecessor_it = dynamic_traversal.parent[successor_it->getLinkIndex()];
            }
                 
            
            //The base is always excited
            if( fixed_base ) {
                //fixed base
                setRobotState(q,dq,ddq,KDL::Twist(grav_direction,KDL::Vector(0.0,0.0,0.0)));
                
            } else {
                //floating base
                Eigen::Map<Eigen::Vector3d>(a.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(a.rot.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.rot.data) = M_PI*Eigen::Vector3d::Random();
                
                if( static_regressor ) {
                    //In the case of the static regressor, only the acceleration (i.e. gravitational acceleration) is random
                    SetToZero(a.rot);
                    SetToZero(v.vel);
                    SetToZero(v.rot);
                }
                
                setRobotState(q,dq,ddq,v,a);
            }
            
            int ret_value = computeRegressor(regressor,kt);
            if( ret_value != 0 ) { if( verbose ) std::cout <<  "computeSparseNumericalIdentifiableSubspace fatal error" << std::endl; return -2; }
            
            //For computing the sparse decomposition, we have to remove the line of the regressor of sensor after the considered link
            //Should be better to avoid computing this lines, but for now it easier to elimate them
            std::vector<bool> keep_regressor(regressors_ptrs.size());
            int reduced_regressor_rows = 0;
            for(int regr = 0; regr < regressors_ptrs.size(); regr++ ) {
                bool keep_regressor_rows = false;
                DynamicRegressorInterface * considered_regr = regressors_ptrs[regr];
                
                std::vector<int> relative_junctions = considered_regr->getRelativeJunctions();
                
                if( relative_junctions.size() == 0 ) { 
                    //The regressor is a global one and is not related to some specific junction
                    keep_regressor_rows = true;
                } 
                
                for(int relative_junction_id = 0; relative_junction_id < relative_junctions.size(); relative_junction_id++ ) {
                    int relative_junction = relative_junctions[relative_junction_id];
                    if( considered_junctions[relative_junction] ) {
                        //The junction of the regressor is before the considered link, keeping the regressor
                        keep_regressor_rows = true;
                    }
                }
                
                keep_regressor[regr] = keep_regressor_rows;
                
                if( keep_regressor_rows ) {
                    reduced_regressor_rows += considered_regr->getNrOfOutputs();
                }
                
            }
            
            Eigen::MatrixXd reduced_regressor = Eigen::MatrixXd(reduced_regressor_rows,regressor.cols());
            
            int reduced_regressor_rows_offset = 0;
            int regressor_rows_offset = 0;
            for(int regr = 0; regr < regressors_ptrs.size(); regr++ ) {
                if( keep_regressor[regr] ) { 
                    reduced_regressor.block(reduced_regressor_rows_offset,0,regressors_ptrs[regr]->getNrOfOutputs(),regressor.cols()) =
                        regressor.block(regressor_rows_offset,0,regressors_ptrs[regr]->getNrOfOutputs(),regressor.cols());
                        
                    reduced_regressor_rows_offset += regressors_ptrs[regr]->getNrOfOutputs();
                }
                
                regressor_rows_offset += regressors_ptrs[regr]->getNrOfOutputs();
            }
            
            if( sample == 0 ) {
                A = reduced_regressor.transpose()*reduced_regressor;
            } else {
                A += reduced_regressor.transpose()*reduced_regressor;
            }
        }
        
        std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~A~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        getRowSpaceBasis(A,dense_basis);        
        std::cout<<"~~~~~~~~~~~~~~~~~~~~~obtained dense basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        std::cout << dense_basis << std::endl;
        
        if( i == 0 ) {
            #ifndef NDEBUG
            std::cout << "Adding link " << link_it->getName() << " adds " << dense_basis.cols() << " base parameters" << std::endl;
            #endif
            sparse_basis = dense_basis;
        } else {
            // \todo check if the dense basis is empty (fake base link) 
            //Update sparse_basis 
            //Get the number of base parameters
            assert( sparse_basis.rows() == dense_basis.rows());
            assert( sparse_basis.rows() == getNrOfParameters());
            int old_nbp = sparse_basis.cols();
            
            
            //project the new dense_basis on the nullspace of the sparse_matrix
            Eigen::MatrixXd nullspace_dense_basis =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*dense_basis;
            
            Eigen::MatrixXd check_nullspace =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*sparse_basis;
            
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << sparse_basis << std::endl;
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~null_space_dense_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << nullspace_dense_basis << std::endl;
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~check_nullspace~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << check_nullspace << std::endl;
            
            //The rank of nullspace_dense_basis is then by definition new_nbp-old_nbp
            //It is then possible to get the row space basis of nullspace_dense_basis, and to add this vector to the sparse_basis
            Eigen::MatrixXd new_sparse_basis;
            
            std::cout << "~~~~~~~~~~~~~getRowSpaceBasis(nullspace_dense_basis,new_sparse_basis)~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            getRowSpaceBasis(nullspace_dense_basis.transpose(),new_sparse_basis);
            std::cout << "~~~~~~~~~~~~~~~new_sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << new_sparse_basis << std::endl;
            
            int new_bp = new_sparse_basis.cols();
            
            #ifndef NDEBUG
            std::cout << "Adding link " << link_it->getName() << " adds " << new_bp << " base parameters for a total of " << old_nbp+new_bp << std::endl;
            #endif
            
            sparse_basis.conservativeResize(np,old_nbp+new_bp);
            
            sparse_basis.block(0,old_nbp,new_sparse_basis.rows(),new_sparse_basis.cols()) = new_sparse_basis;
        }
        
    }
    
    basis = sparse_basis;
    
    std::cout << "SparseNumericalFinalTest" << std::endl;
    Eigen::MatrixXd dummy;
    getRowSpaceBasis(basis,dummy);
    
        
    return 0;
}
*/


}
