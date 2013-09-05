/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
 
 
#include <drrl/dynamicRegressorGenerator.hpp>

#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

namespace DRRL
{
 
DynamicRegressorGenerator::DynamicRegressorGenerator(KDL::Tree & tree, std::string kinematic_base,
                                                      std::vector< std::string > & ft_sensor_names, 
                                                      bool ft_sensor_offset,
                                                      KDL::CoDyCo::TreeSerialization serialization): consider_ft_offset(ft_sensors_vector), verbose(false)
{
    tree_graph = KDL::CoDyCo::TreeGraph(_tree,serialization);  
    NrOfDOFs = _tree.getNrOfJoints();
    NrOfLinks = _tree.getNrOfSegments();
    NrOfFTSensors = ft_sensor_names.size();
    
    //The initial number of parameters is given by the inertial parameters
    NrOfParameters = 10*NrOfLinks;  
    
    //If the ft sensor offset is activated, we have to add 6 parameters (the offsets) for each ft sensor
    if( consider_ft_offset ) {
        NrOfParameters += 6*NrOfFTSensors;
    }
    
    //Initially no regressor is installed, so the number of outputs is zero
    NrOfOutputs = 0;
    
    assert((int)tree_graph.getNrOfDOFs() == NrOfDOFs);
    assert((int)tree_graph.getNrOfLinks() == NrOfLinks);
    
    q = KDL::JntArray(NrOfDOFs);
    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);
    
    measured_torques = KDL::JntArray(NrOfDOFs);
    
    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();
    
    measured_wrenches =  std::vector< KDL::Wrench >(NrOfFTSensors);
    
    X_dynamic_base = std::vector<KDL::Frame>(NrOfLinks);
    v = std::vector<KDL::Twist>(NrOfLinks);
    a = std::vector<KDL::Twist>(NrOfLinks);
    
    //Computing the traversal for kinematic information
    ret = tree_graph.compute_traversal(kinematic_traversal,kinematic_base);
    assert( ret >= 0);
    if( ret < 0 ) { return; }
    
    //Computing the default (dynamic) traversal
    ret = tree_graph.compute_traversal(dynamic_traversal);
    assert( ret >= 0 );
    if( ret < 0 ) { return; }
    
    ft_list = KDL::CoDyCo::FTSensorList(tree_graph,ft_sensors_names);
    //Everything ok?
    assert( ft_list.getNrOfFTSensors() == NrOfFTSensors );
}

int DynamicRegressorGenerator::getNrOfParameters()
{
    return NrOfParameters;
}
    
int DynamicRegressorGenerator::getNrOfOutputs()
{
    return NrOfOutputs;
}

std::string DynamicRegressorGenerator::getDescriptionOfParameter(int parameter_index)
{
    std::stringstream ss;
    
    if( parameter_index >= NrOfParameters ) {
        ss << "DynamicRegressorGenerator::getDescriptionOfParameter error: parameter_index " << parameter_index << " is greater the number of parameters " << NrOfParameters;
        if( verbose ) { std::cerr << ss.str() << std::endl; }
        return ss.str();
    }
    
    //The first 10*NrOfLinks parameters are always the inertial ones
    if( parameter_index < 10*NrOfLinks ) {
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
        link_index = parameter_index/10;
        ss << "Parameter " << parameter_index << ": " << inertial_parameter_type << " of link " << tree_graph.getLink(link_index)->getName() << " (" << link_index << ")";
    } 
    //else
    //if the offset are present, they are the parameters right after the inertial parameters 
    parameter_index_offset = parameter_index - 10*NrOfLinks;
    
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
        ft_sensor_index = parameter_index_offset/6;
        ss << "Parameter " << parameter_index << ": " << ft_offset_type << " of link " << ft_list.ft_sensors_vector[ft_sensor_index].getName() << " (" << ft_sensor_index << ")";
    }
    
    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfParameters()
{
    std::stringstream ss;
    
    for(parameter_index=0; paremeter_index<NrOfParameters; parameter_index++) {
        ss << getDescriptionOfParameter(parameter_index) << std::endl;
    }
    
    return ss.str();
}
     
int DynamicRegressorGenerator::setRobotState(const JntArray &_q, const JntArray &_q_dot, const JntArray &_q_dotdot, const Twist& _base_velocity, const Twist& _base_acceleration)
{
    if( _q.rows() != NrOfDOFs() || _q_dot.rows() != NrOfDOFs || _q_dotdot.rows() != NrOfDOFs ) {
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

int DynamicRegressorGenerator::setRobotState(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_gravity)
{
    KDL::Twist dummy = KDL::Twist::Zero();
    return setRobotState(q,q_dot,q_dotdot,dummy,base_gravity);
}

int DynamicRegressorGenerator::computeRegressor(MatrixXd & regressor, VectorXd & known_terms)
{
    if( regressor.rows() != getNrOfOutputs() || regressor.cols() != getNrOfParameters() || known_terms.size() != getNrOfOutputs )
    {
        if( verbose ) { std::cerr << "DynamicsRegressorGenerator::computeRegressor error: input size errror" << std::endl; }
        return -1;
    }
    
    //Calculating the velocity and acceleration for each link
    rneaKinematicLoop(tree_graph,q,dq,ddq,kinematic_traversal,kinematic_base_velocity,kinematic_base_acceleration,v,a);
    
    //Get the frame between each link and the base
    getFramesLoop(tree_graph,q,dynamic_traversal,X_dynamic_base);
    
    //Called specific regressors
    
    
}


}
