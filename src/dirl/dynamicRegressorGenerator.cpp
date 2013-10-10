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

namespace dirl
{
 
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
    int ret = tree_graph.compute_traversal(kinematic_traversal,kinematic_base);
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

int DynamicRegressorGenerator::computeIdentifiableSubspace(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    if( n_samples < 0 ) return -1;
        int no = getNrOfOutputs();
        int np = getNrOfParameters();
        int nj = NrOfDOFs;
        
        //sketch ! not working
        Eigen::MatrixXd A(np,np); //working matrix
        Eigen::MatrixXd regressor(no,np);
        Eigen::VectorXd kt(no);
    
        KDL::JntArray q(nj),dq(nj),ddq(nj);
        KDL::Twist a,v;
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
        
        //Probably can be improved using a different decomposition
        //NOT REAL TIME!!
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        sigma = svd.singularValues();
        
        if(verbose) {  std::cout << "Singular values " << std::endl; std::cout << sigma << std::endl; }
        
        if( tol <= 0 ) {
            tol = sigma[0]*n_samples*no*DBL_EPSILON;
        }
        
        int ll;
        for(ll=0; ll < sigma.size(); ll++ ) {
            if( sigma[ll] < tol ) { break;}
        }
      
        int rank = ll;
         
        V = svd.matrixV();
        assert(V.cols() == V.rows());
        if( V.cols() != V.rows() ) { std::cout << "V is not square" << std::endl; }
        assert(rank <= np);
        basis.resize(np,rank);
        if( verbose ) { std::cout << "Matrix has rank " << rank << std::endl; std::cout << " np " << np << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl; }

        basis = V.block(0,0,np,rank);
        
        if( verbose ) { std::cout << "basis calculated, tol calculate " << DBL_EPSILON*np*sigma[0] << std::endl; }
        //NonBaseSpaceBasis = V.block(0,rank+1,np,np-rank);
        
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

int DynamicRegressorGenerator::updateBuffers()
{
    one_rows_buffer = Eigen::MatrixXd(1,NrOfParameters);
    six_rows_buffer = Eigen::MatrixXd(6,NrOfParameters);
    
    one_rows_vector = Eigen::VectorXd(1);
    six_rows_vector = Eigen::VectorXd(6);
    
    return -1;
}
    




}
