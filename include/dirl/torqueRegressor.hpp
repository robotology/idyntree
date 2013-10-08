/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */

#ifndef _DIRL_TORQUE_REGRESSOR_
#define _DIRL_TORQUE_REGRESSOR_

namespace dirl 
{

class torqueRegressor : public DynamicRegressorInterface 
{   
    std::string torque_dof;
    int torque_dof_index;
    
    std::vector<bool> activated_ft_sensors;
    
    public:
        torqueRegressor(const std::string & dof_name, const std::vector<bool> & _activated_ft_sensors=std::vector< bool>(0)):torque_dof(dof_name), activated_ft_sensors(_activated_ft_sensors) {};
        
        ~torqueRegressor() {};
        
        int getNrOfOutputs();
        
        int compute_regressor(const KDL::JntArray &q, 
                                  const KDL::JntArray &q_dot, 
                                  const KDL::JntArray &q_dotdot,
                                  const std::vector<KDL::Frame> & X_dynamic_base,
                                  const std::vector<KDL::Twist> v,
                                  const std::vector< KDL::Wrench > & measured_wrenches,
                                  const KDL::JntArray measured_torques,
                                  Eigen::MatrixXd & regressor_matrix,
                                  Eigen::VectorXd & known_terms);
    
};

}
#endif
