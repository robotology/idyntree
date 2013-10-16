/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
 
#include <cmath>
#include <cfloat>

#include <dirl/dynamicRegressorGenerator.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <iostream>

#include <algorithm>
#include <deque>

namespace dirl
{
 
double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}    
    
DynamicRegressorGenerator::DynamicRegressorGenerator(KDL::Tree & _tree, std::string kinematic_base,
                                                      std::vector< std::string > ft_sensor_names, 
                                                      bool ft_sensor_offset,
                                                      std::vector< std::string > fake_links_names, 
                                                      KDL::CoDyCo::TreeSerialization serialization): 
                                                      consider_ft_offset(ft_sensor_offset),
                                                      regressors_ptrs(0),
                                                      verbose(true)
{
#ifndef NDEBUG
    std::cout << "called DynamicRegressorGenerator with " << ft_sensor_names.size() << " ft sensor " <<  std::endl;
#endif
    tree_graph = KDL::CoDyCo::TreeGraph(_tree,serialization);  
    NrOfFakeLinks = fake_links_names.size();
    NrOfDOFs = _tree.getNrOfJoints();
    NrOfRealLinks_gen = _tree.getNrOfSegments()-NrOfFakeLinks;
    NrOfFTSensors = ft_sensor_names.size();
    
    //The initial number of parameters is given by the inertial parameters
    NrOfParameters = 10*NrOfRealLinks_gen;  
    
    //If the ft sensor offset is activated, we have to add 6 parameters (the offsets) for each ft sensor
    if( consider_ft_offset ) {
        NrOfParameters += 6*NrOfFTSensors;
    }
    
    //Given the NrOfParameters, allocate buffers
    updateBuffers();
    
    //Initially no regressor is installed, so the number of outputs is zero
    NrOfOutputs = 0;
    
    //
    //torque_regressors(0),

    
    assert((int)tree_graph.getNrOfDOFs() == NrOfDOFs);
    assert((int)tree_graph.getNrOfLinks() == NrOfFakeLinks+NrOfRealLinks_gen);
    
    q = KDL::JntArray(NrOfDOFs);
    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);
    
    measured_torques = KDL::JntArray(NrOfDOFs);
    
    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();
    
    measured_wrenches =  std::vector< KDL::Wrench >(NrOfFTSensors);
    
    X_dynamic_base = std::vector<KDL::Frame>(tree_graph.getNrOfLinks());
    v = std::vector<KDL::Twist>(tree_graph.getNrOfLinks());
    a = std::vector<KDL::Twist>(tree_graph.getNrOfLinks());
    
    //Computing the traversal for kinematic information
    int ret;
    if( kinematic_base.length() == 0 ) {
        //default case, using the base of the tree as the kinematic base 
        ret = tree_graph.compute_traversal(kinematic_traversal);
    } else {
        ret = tree_graph.compute_traversal(kinematic_traversal,kinematic_base);
    }
    assert( ret >= 0);
    if( ret < 0 ) { return; }
    
    //Computing the default (dynamic) traversal
    ret = tree_graph.compute_traversal(dynamic_traversal);
    assert( ret >= 0 );
    if( ret < 0 ) { return; }
    
    //ft_list = KDL::CoDyCo::FTSensorList(tree_graph,ft_sensor_names);
    p_ft_list = new KDL::CoDyCo::FTSensorList(tree_graph,ft_sensor_names);
    
    //Everything ok?
    assert( p_ft_list->getNrOfFTSensors() == NrOfFTSensors );
    
    //Take into acount the real or fake links
    is_link_real.resize(tree_graph.getNrOfLinks(),true);
    linkIndeces2regrColumns.resize(tree_graph.getNrOfLinks(),-1);
    regrColumns2linkIndeces.resize(NrOfRealLinks_gen,-1);
    for(int ll=0; ll < fake_links_names.size(); ll++ ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it = tree_graph.getLink(fake_links_names[ll]);
        if( link_it == tree_graph.getInvalidLinkIterator() ) { NrOfDOFs = NrOfRealLinks_gen = NrOfOutputs = NrOfParameters = 0; return; }
        is_link_real[link_it->getLinkIndex()]=false;
    }
    
    int regressor_link_index = 0;
    for(int link_index=0; link_index < tree_graph.getNrOfLinks(); link_index++ ) {
        if( is_link_real[link_index] ) {
            assert( regressor_link_index < NrOfRealLinks_gen );
            linkIndeces2regrColumns[link_index] = regressor_link_index;
            regrColumns2linkIndeces[regressor_link_index] = link_index;
            regressor_link_index++;
        } else {
           linkIndeces2regrColumns[link_index] = -1;
        }
    }
    assert(regressor_link_index == NrOfRealLinks_gen);
}

int DynamicRegressorGenerator::getNrOfParameters()
{
    return NrOfParameters;
}
    
int DynamicRegressorGenerator::getNrOfOutputs()
{
    return NrOfOutputs;
}



std::string DynamicRegressorGenerator::getDescriptionOfParameter(int parameter_index, bool with_value, double value)
{
#ifndef NDEBUG
    //std::cout << "GetdescriptionOfParameter with argument " << parameter_index << std::endl;
#endif
    std::stringstream ss;
    
    if( parameter_index >= NrOfParameters ) {
        ss << "DynamicRegressorGenerator::getDescriptionOfParameter error: parameter_index " << parameter_index << " is greater the number of parameters " << NrOfParameters;
        if( verbose ) { std::cerr << ss.str() << std::endl; }
        return ss.str();
    }
    
    //The first 10*NrOfLinks parameters are always the inertial ones
    if( parameter_index < 10*NrOfRealLinks_gen ) {
        std::string inertial_parameter_type;
        switch( parameter_index % 10 ) {
            case 0:
            inertial_parameter_type = "mass";
            break;
            case 1:
            inertial_parameter_type = "x component of first moment of mass";
            break;
            case 2:
            inertial_parameter_type = "y component of first moment of mass";
            break;
            case 3:
            inertial_parameter_type = "z component of first moment of mass";
            break; 
            case 4:
            inertial_parameter_type = "xx component of inertia matrix";
            break;
            case 5:
            inertial_parameter_type = "xy component of inertia matrix";
            break;
            case 6:
            inertial_parameter_type = "xz component of inertia matrix";
            break;
            case 7:
            inertial_parameter_type = "yy component of inertia matrix";
            break;
            case 8:
            inertial_parameter_type = "yz component of inertia matrix";
            break;
            case 9:
            inertial_parameter_type = "zz component of inertia matrix";
            break;
            
        }
        int link_index = parameter_index/10;
        ss << "Parameter " << parameter_index << ": " << inertial_parameter_type << " of link " << tree_graph.getLink(regrColumns2linkIndeces[link_index])->getName() << " (" << regrColumns2linkIndeces[link_index] << ")";

    } else { 
    //else
    //if the offset are present, they are the parameters right after the inertial parameters 
    int parameter_index_offset = parameter_index - 10*NrOfRealLinks_gen;
    
    if( consider_ft_offset ) {
        std::string ft_offset_type;
         switch( parameter_index_offset % 6 ) {
            case 0:
            ft_offset_type = "x component of force offset";
            break;
            case 1:
            ft_offset_type = "y component of force offset";
            break;
            case 2:
            ft_offset_type = "z component of force offset";
            break;
            case 3:
            ft_offset_type = "x component of torque offset";
            break; 
            case 4:
            ft_offset_type = "y component of torque offset";
            break;
            case 5:
            ft_offset_type = "z component of torque offset";
            break;
        }
        int ft_sensor_index = parameter_index_offset/6;

        assert(ft_sensor_index < NrOfFTSensors);
        assert(p_ft_list->ft_sensors_vector.size() == NrOfFTSensors);
        
        ss << "Parameter " 
           << parameter_index 
           << ": " << ft_offset_type << " of link " 
           << p_ft_list->ft_sensors_vector[ft_sensor_index]->getName() << 
           " (" << ft_sensor_index << ")";
    }
    }
    
    if( with_value ) { 
        ss << "\t\t" << value << std::endl;
    }
    
    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfParameters()
{
    std::stringstream ss;
    
    for(int parameter_index=0; parameter_index<NrOfParameters; parameter_index++) {
        ss << getDescriptionOfParameter(parameter_index) << std::endl;
    }
    
    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfParameters(const Eigen::VectorXd & values)
{
    std::stringstream ss;
    
    for(int parameter_index=0; parameter_index<NrOfParameters; parameter_index++) {
        ss << getDescriptionOfParameter(parameter_index,true,values[parameter_index]) << std::endl;
    }
    
    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfOutput(int output_index)
{
    std::stringstream ss;
    
    ss << "DynamicRegressorGenerator::getDescriptionOfOutput(" << output_index << ") : Not implemented.";
    
    return ss.str();
}
    
std::string DynamicRegressorGenerator::getDescriptionOfOutputs()
{
    std::stringstream ss;
    
    for(int output_index=0; output_index < NrOfOutputs; output_index++) {
        ss << getDescriptionOfOutput(output_index) << std::endl;
    }
    
    return ss.str();
}
     
int DynamicRegressorGenerator::setRobotState(const KDL::JntArray &_q, const KDL::JntArray &_q_dot, const KDL::JntArray &_q_dotdot, const KDL::Twist& _base_velocity, const KDL::Twist& _base_acceleration)
{
    if( _q.rows() != NrOfDOFs || _q_dot.rows() != NrOfDOFs || _q_dotdot.rows() != NrOfDOFs ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::setRobotState error: input size error" << std::endl; }
        return -1;
    }
    q = _q;
    dq = _q_dot;
    ddq = _q_dotdot;
    kinematic_base_velocity = _base_velocity;
    kinematic_base_acceleration = _base_acceleration;;
    return 0;
}

int DynamicRegressorGenerator::setRobotState(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Twist& base_gravity)
{
    KDL::Twist dummy = KDL::Twist::Zero();
    return setRobotState(q,q_dot,q_dotdot,dummy,base_gravity);
}

int DynamicRegressorGenerator::computeRegressor( Eigen::MatrixXd & regressor, Eigen::VectorXd & known_terms)
{
    if( regressor.rows() != getNrOfOutputs() || regressor.cols() != getNrOfParameters() || known_terms.size() != getNrOfOutputs() )
    {
        if( verbose ) { std::cerr << "DynamicsRegressorGenerator::computeRegressor error: input size error" << std::endl; }
        return -1;
    }
    
    //Calculating the velocity and acceleration for each link
    rneaKinematicLoop(tree_graph,q,dq,ddq,kinematic_traversal,kinematic_base_velocity,kinematic_base_acceleration,v,a);
    
    //Get the frame between each link and the base
    getFramesLoop(tree_graph,q,dynamic_traversal,X_dynamic_base);
    
    //Call specific regressors
    int start_row = 0;
    for(int i=0; i < regressors_ptrs.size(); i++ ) {
        DynamicRegressorInterface * regr_ptr = regressors_ptrs[i];
        
        /**
         * Little workaround to avoid dynamic allocation of memory
         * 
         * \todo this use of fixed buffers avoid this by using something similar to Eigen::Ref (unfortunately available only on Eigen 3.2.0) 
         */
       
        switch( regr_ptr->getNrOfOutputs() )
        {
            case 6: 
                assert(six_rows_buffer.cols() == 10*NrOfRealLinks_gen+6*NrOfFTSensors); 
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,measured_wrenches,measured_torques,six_rows_buffer,six_rows_vector);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = six_rows_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = six_rows_vector;
            break;
            case 1:
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,measured_wrenches,measured_torques,one_rows_buffer,one_rows_vector);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = one_rows_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = one_rows_vector;
            break;
            default:
                //This should not be used,however a dynamic memory solution is added 
                //as a fallback in case another type of regressor is added without modifyng this code
                Eigen::MatrixXd regr_buffer(regr_ptr->getNrOfOutputs(),getNrOfParameters());
                Eigen::VectorXd kt_buffer(regr_ptr->getNrOfOutputs());
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,measured_wrenches,measured_torques,regr_buffer,kt_buffer);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = regr_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = kt_buffer;
            break;
        }
        
        start_row += regr_ptr->getNrOfOutputs();
    }
    
    return 0;

}

/*
class FloatingBaseRobotState
{
    public:
        JntArray q;
        JntArray dq;
        JntArray ddq;
}*/



int DynamicRegressorGenerator::getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol)
{
        std::cout << "Called getRowSpaceBasis with input_matrix: " << std::endl;
        std::cout << input_matrix << std::endl;
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
        
        row_space_basis_matrix.resize(m,rank);
        std::cout << "Matrix has rank " << rank << std::endl; std::cout << " m " << m << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl; 
        row_space_basis_matrix = V.block(0,0,m,rank);
        std::cout << "basis calculated, tol calculate " << tol << std::endl; 
        
        return 0;
}

int DynamicRegressorGenerator::computeNumericalIdentifiableSubspace(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    if( n_samples < 0 ) return -1;
        int no = getNrOfOutputs();
        int np = getNrOfParameters();
        int nj = NrOfDOFs;
    

        //Generated robot state
        KDL::JntArray q(nj),dq(nj),ddq(nj);
        KDL::Twist a,v;
    
    
        //sketch ! not working
        //Convenience variables
        Eigen::MatrixXd A(np,np); //working matrix
        Eigen::MatrixXd regressor(no,np);
        Eigen::VectorXd kt(no);
    
        Eigen::MatrixXd V(np,np);
        Eigen::VectorXd sigma(np);

        /** \todo store and restore class state */
        
        for(int i=0; i < n_samples; i++ ) {
            //RegressorSolver
            //Excite with random data RegressorSolver (random q, dq, ddq (for energy, simply do not use ddq)
            /** \todo use limits of angles */
            q.data = M_PI*Eigen::VectorXd::Random(nj);
            if( static_regressor ) {
                SetToZero(dq);
                SetToZero(ddq);
            } else {
                dq.data = M_PI*Eigen::VectorXd::Random(nj);
                ddq.data = M_PI*Eigen::VectorXd::Random(nj);
            }
            
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

                }
                
                
                setRobotState(q,dq,ddq,v,a);
            }
            
            int ret_value = computeRegressor(regressor,kt);
            if( ret_value != 0 ) { if( verbose ) std::cout <<  "computeIdentifiableSubspace fatal error" << std::endl; return -2; }
            
            
            
            
            if( i == 0 ) {
                A = regressor.transpose()*regressor;
            } else {
                A += regressor.transpose()*regressor;
            }
        }
        
        return getRowSpaceBasis(A,basis);
        
}

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
    std::vector<bool> considered_junctions(tree_graph.getNrOfJunctions());
    
    //Calculate sequentially the subspace base, adding a link at the time
    for(int i=0; i < tree_graph.getNrOfLinks(); i++ ) {
        
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
            
            while( predecessor_it != tree_graph.getInvalidLinkIterator() ) {
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
            /** \todo check if the dense basis is empty (fake base link) */
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


int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspace(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
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
    std::vector<bool> considered_junctions(tree_graph.getNrOfJunctions());
    
    //Calculate sequentially the subspace base, considering a subtree starting at a DOF at each loop
    for(int i=tree_graph.getNrOfLinks()-1; i >=0; i-- ) {
        
        /////////////////////////////////////////
        /// Subspace till a given tree
        /////////////////////////////////////////
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.order[i];
        
        #ifndef NDEBUG
        std::cerr << "Generating samples for all DOFs " << link_it->getName() << std::endl;
        #endif
        
        for(int sample=0; sample < n_samples; sample++ ) {
            //generated suitable state, exciting only the dofs before the current considered link
            
            //Initially set to zero all the coordinates, then generated random only for the dof before the considered link
            q.data = M_PI*Eigen::VectorXd::Random(nj);
            if( static_regressor ) {
                SetToZero(dq);
                SetToZero(ddq);
            } else {
                dq.data = M_PI*Eigen::VectorXd::Random(nj);
                ddq.data = M_PI*Eigen::VectorXd::Random(nj);
            }
            
            
            //Enumerate all the DOFs and junctions after the considered link
            //By default none of them
            fill(considered_junctions.begin(),considered_junctions.end(),false);
            std::deque<int> link_to_visit;
            link_to_visit.push_back(link_it->getLinkIndex());
            
            while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = tree_graph.getLink(considered_link_index);
                
                for(int child=0; child < considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);
                    
                    if( considered_next_link == dynamic_traversal.parent[considered_link_index] ) {
                        //This is the parent of the link, already considered
                        continue;
                    }
                    
                    considered_junctions[considered_junction->getJunctionIndex()] = true;
                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }
                
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
                    //So it is considered only when the last link is added
                    keep_regressor_rows = false;

                } 
                
                if( relative_junctions.size() == 1 ) {
                    if( considered_junctions[relative_junctions[0]] ) {
                        keep_regressor_rows = true;
                    }
                    
                } else {
                    
                    for(int relative_junction_id = 0; relative_junction_id < relative_junctions.size(); relative_junction_id++ ) {
                        int relative_junction = relative_junctions[relative_junction_id];
                        //If all the junction of the regressor are considered, keep the regressor
                        if( considered_junctions[relative_junction] ) {
                            keep_regressor_rows = true;
                        } else {
                           keep_regressor_rows = false;
                           break;
                        }
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
        
        if( i == tree_graph.getNrOfLinks()-1 ) {
            #ifndef NDEBUG
            std::cout << "Adding link " << link_it->getName() << " adds " << dense_basis.cols() << " base parameters" << std::endl;
            #endif
            sparse_basis = dense_basis;
        } else {
            /** \todo check if the dense basis is empty (fake base link) */
            //Update sparse_basis 
            //Get the number of base parameters
            assert( (sparse_basis.rows() == dense_basis.rows()) || (sparse_basis.cols() == 0));
            assert( (sparse_basis.rows() == getNrOfParameters())  || (sparse_basis.cols() == 0));
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
    
    //Adding an iteration in which all the other regressor are considered
    {
        int old_nbp = sparse_basis.cols();

        Eigen::MatrixXd dense_basis;
        
        computeNumericalIdentifiableSubspace(dense_basis,static_regressor,fixed_base, grav_direction, tol,n_samples, verbose);
        
        Eigen::MatrixXd nullspace_dense_basis =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*dense_basis;

        //The rank of nullspace_dense_basis is then by definition new_nbp-old_nbp
        //It is then possible to get the row space basis of nullspace_dense_basis, and to add this vector to the sparse_basis
        Eigen::MatrixXd new_sparse_basis;
            
        std::cout << "~~~~~~~~~~~~~getRowSpaceBasis(nullspace_dense_basis,new_sparse_basis)~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            getRowSpaceBasis(nullspace_dense_basis.transpose(),new_sparse_basis);
            std::cout << "~~~~~~~~~~~~~~~new_sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << new_sparse_basis << std::endl;
            
            int new_bp = new_sparse_basis.cols();
            
            #ifndef NDEBUG
            std::cout << "Adding all the other regressors adds " << new_bp << " base parameters for a total of " << old_nbp+new_bp << std::endl;
            #endif
            
            sparse_basis.conservativeResize(np,old_nbp+new_bp);
            
            sparse_basis.block(0,old_nbp,new_sparse_basis.rows(),new_sparse_basis.cols()) = new_sparse_basis;

    }
    
    basis = sparse_basis;
    
    std::cout << "SparseNumericalFinalTest" << std::endl;
    Eigen::MatrixXd dummy;
    getRowSpaceBasis(basis,dummy);
    
        
    return 0;
}

int DynamicRegressorGenerator::setFTSensorMeasurement(const int ft_sensor_index, const KDL::Wrench ftm)
{
    if( ft_sensor_index < 0 || ft_sensor_index >= NrOfFTSensors ) {
        if( verbose ) std::cerr << "DynamicsRegressorGenerator::setFTSensorMeasurement error: ft_sensor_index " << ft_sensor_index  << " out of bounds" << std::endl;
    
        return -1;
    }
    
    measured_wrenches[ft_sensor_index] = ftm;
    
    return 0;
}
    
int DynamicRegressorGenerator::setTorqueSensorMeasurement(const int dof_index, const double measure)
{
    if( dof_index < 0 || dof_index >= NrOfDOFs ) {
        if( verbose ) std::cerr << "DynamicsRegressorGenerator::setTorqueSensorMeasurement error: dof_index " << dof_index  << " out of bounds" << std::endl;
        return -1;
    }
    
    measured_torques(dof_index) = measure;
    
    return 0;
}

int DynamicRegressorGenerator::setTorqueSensorMeasurement(const KDL::JntArray & torques)
{
    if( torques.rows() != measured_torques.rows() ) {
        if( verbose ) std::cerr << "DynamicsRegressorGenerator::setTorqueSensorMeasurement error" << std::endl;
        return -1;
    }
    
    measured_torques = torques;
    
    return 0;
}

  
int DynamicRegressorGenerator::addSubtreeRegressorRows(const std::vector< std::string>& _subtree_leaf_links)
{
    DynamicRegressorInterface * new_regr;
    subtreeBaseDynamicsRegressor * new_st_regr;
    
    new_st_regr = new subtreeBaseDynamicsRegressor(tree_graph,*p_ft_list,linkIndeces2regrColumns,_subtree_leaf_links,consider_ft_offset,verbose);
    
    new_regr = (DynamicRegressorInterface *) new_st_regr;
    
    int ret_val = new_st_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addSubtreeRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_st_regr;
        return -1;
    }
   
    subtree_regressors.push_back(new_st_regr);
    regressors_ptrs.push_back(new_regr);
    
    NrOfOutputs += new_regr->getNrOfOutputs();
    
    return 0;
}


int DynamicRegressorGenerator::addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction, const std::vector<bool> &_activated_ft_sensors)
{
    DynamicRegressorInterface * new_regr;
    torqueRegressor * new_st_regr;
    
    new_st_regr = new torqueRegressor(tree_graph,*p_ft_list,linkIndeces2regrColumns,dof_name,reverse_direction,_activated_ft_sensors,consider_ft_offset,verbose);
    
    new_regr = (DynamicRegressorInterface *) new_st_regr;
   
    int ret_val = new_st_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_st_regr;
        return -1;
    }
    
    torque_regressors.push_back(new_st_regr);
    regressors_ptrs.push_back(new_regr);
    
    NrOfOutputs += new_regr->getNrOfOutputs();
    
    return 0;
} 


int DynamicRegressorGenerator::addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction, const std::vector<std::string> &_activated_ft_sensors)
{
    std::vector<bool> flag_activated_ft_sensors(p_ft_list->getNrOfFTSensors(),false);
    for(int i=0; i < _activated_ft_sensors.size(); i++ ) {
       KDL::CoDyCo::LinkMap::const_iterator link_it =  tree_graph.getLink(_activated_ft_sensors[i]);
       if( link_it == tree_graph.getInvalidLinkIterator() ) {
            if( verbose ) { std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error: link " << _activated_ft_sensors[i]  << " does not exists" << std::endl; }
            return -1;
       }
       std::vector<const KDL::CoDyCo::FTSensor*> ft_on_link = p_ft_list->getFTSensorsOnLink(link_it->getLinkIndex());
       for(int l=0; l < ft_on_link.size(); l++ ) {
           int ft_id = ft_on_link[l]->getID();
           if( ft_id >= flag_activated_ft_sensors.size() || ft_id < 0 ) {
               if( verbose ) { std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error while adding subtree with leaf: " << _activated_ft_sensors[i]  << std::endl; }
               return -1;
           }
           flag_activated_ft_sensors[ft_id] = true;
       }
    }
    return addTorqueRegressorRows(dof_name,reverse_direction,flag_activated_ft_sensors);
}


int DynamicRegressorGenerator::addAllTorqueRegressorRows()
{
    int ret_val;
    for(int i=0; i < tree_graph.getNrOfDOFs(); i++ ) {
        ret_val = addTorqueRegressorRows(tree_graph.getJunction(i)->getName());
        if( ret_val != 0 ) { return ret_val; }
    }
    return 0;
}

int DynamicRegressorGenerator::addBaseRegressorRows()
{
    DynamicRegressorInterface * new_regr;
    baseDynamicsRegressor * new_base_regr;
    
    new_base_regr = new baseDynamicsRegressor(tree_graph,linkIndeces2regrColumns,verbose);
    
    new_regr = (DynamicRegressorInterface *) new_base_regr;
    
    int ret_val = new_base_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addBaseRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_base_regr;
        return -1;
    }
   
    base_regressors.push_back(new_base_regr);
    regressors_ptrs.push_back(new_regr);
    
    NrOfOutputs += new_regr->getNrOfOutputs();
    
    return 0;
}

int DynamicRegressorGenerator::updateBuffers()
{
    one_rows_buffer = Eigen::MatrixXd(1,NrOfParameters);
    six_rows_buffer = Eigen::MatrixXd(6,NrOfParameters);
    
    one_rows_vector = Eigen::VectorXd(1);
    six_rows_vector = Eigen::VectorXd(6);
    
    return -1;
}
    




}
