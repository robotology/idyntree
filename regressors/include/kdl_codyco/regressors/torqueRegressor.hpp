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

#include "dynamicRegressorInterface.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors {
/** \todo fix the case where the dynamic base has changed */
class torqueRegressor : public DynamicRegressorInterface 
{   
    const KDL::CoDyCo::UndirectedTree * p_undirected_tree;
    const KDL::CoDyCo::FTSensorList * p_ft_list;    
            
    std::vector< int > subtree_leaf_links_indeces; /** indices of the leafs (excluding the root) */  
        
    std::vector<int> linkIndeces2regrCols;
    
    std::string torque_dof;
    
    bool reverse_direction;
    
    std::vector<bool> activated_ft_sensors;
    
    bool consider_ft_offset;
    
    std::vector< int > subtree_links_indices; /** indeces of the links belonging to the considered subtree */
    
    bool verbose;
    
    int torque_dof_index;
    
    int subtree_root_link_id;
        
    
    std::vector<int> relative_junction;
    
    int NrOfRealLinks_subtree;

    
    public:
        /** 
         * 
         * @param _reverse_direction if true, reverse the direction of the regressor (root to joint instead of leaf to joint) default:false
         */
        torqueRegressor(const KDL::CoDyCo::UndirectedTree & _undirected_tree, 
                        const KDL::CoDyCo::FTSensorList & _ft_list,
                        const std::vector<int> & _linkIndeces2regrCols,
                        const std::string & dof_name, 
                        const bool _reverse_direction = false,
                        const std::vector<bool> & _activated_ft_sensors=std::vector< bool>(0),
                        const bool _consider_ft_offset=false,
                        const bool _verbose=true
                        )
                        :   p_undirected_tree(&_undirected_tree),
                                            p_ft_list(&_ft_list),
                                            linkIndeces2regrCols(_linkIndeces2regrCols),
                                            torque_dof(dof_name), 
                                            reverse_direction(_reverse_direction), 
                                            activated_ft_sensors(_activated_ft_sensors),
                                            consider_ft_offset(_consider_ft_offset),
                                            subtree_links_indices(0),
                                            verbose(_verbose),
                                            NrOfRealLinks_subtree(0)
                                         
        {
            assert(linkIndeces2regrCols.size() == p_undirected_tree->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < (int)linkIndeces2regrCols.size(); ll++ ) { if( linkIndeces2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= (int)linkIndeces2regrCols.size());
        }
        
        virtual ~torqueRegressor() {};
        
        bool isActiveFTSensor(const int ft_sensor_id) const;
        
        int configure();
        
        int getNrOfOutputs();
        
        std::vector<int> getRelativeJunctions();
        
        int computeRegressor(const KDL::JntArray &q, 
                              const KDL::JntArray &q_dot, 
                              const KDL::JntArray &q_dotdot,
                              const std::vector<KDL::Frame> & X_dynamic_base,
                              const std::vector<KDL::Twist> & v,
                              const std::vector<KDL::Twist> & a,
                              const std::vector< KDL::Wrench > & measured_wrenches,
                              const KDL::JntArray & measured_torques,
                              Eigen::MatrixXd & regressor_matrix,
                              Eigen::VectorXd & known_terms);
    
};

}

}

}
#endif
