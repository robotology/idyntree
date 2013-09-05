/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */

#ifndef _DRRL_DYNAMIC_REGRESSOR_INTERFACE_
#define _DRRL_DYNAMIC_REGRESSOR_INTERFACE_

namespace DRRL 
{

class DynamicRegressorInterface {

public:
    virtual int getNrOfOutputs() = 0;
    
    /**
     * Configure the regressor given a TreeGraph (for example allocating 
     * the necessary datastructures)
     * 
     * Return 0 if all went well, a negative number otherwise
     * 
     * \todo substitute KDL::CoDyCo::FTSensorList with a more general structure
     */
    virtual int configure(const KDL::CoDyCo::TreeGraph & tree_graph, const KDL::CoDyCo::FTSensorList & ft_list);
    
    virtual int compute_regressor(const KDL::CoDyCo::TreeGraph & tree_graph,
                                   const JntArray &q, 
                                   const JntArray &q_dot, 
                                   const JntArray &q_dotdot,
                                   const std::vector<KDL::Frame> & X_dynamic_base,
                                   const std::vector<KDL::Twist> v,
                                   const std::vector< KDL::Wrench > & measured_wrenches;
                                   const KDL::JntArray measured_torques,
                                   Eigen::MatrixXd & regressor_matrix,
                                   Eigen::VectorXd & known_terms);
    
}

}
#endif
