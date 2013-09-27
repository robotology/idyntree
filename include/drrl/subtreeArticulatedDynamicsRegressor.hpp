/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */

#ifndef _DRRL_SUBTREE_ARTICULATED_DYNAMICS_REGRESSOR_
#define _DRRL_SUBTREE_ARTICULATED_DYNAMICS_REGRESSOR_

#include <kdl/jntarray.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include <drrl/dynamicRegressorInterface.hpp>

namespace DRRL 
{

class subtreeArticulatedDynamicsRegressor : public DynamicRegressorInterface 
{
    const KDL::CoDyCo::TreeGraph * p_tree_graph;
    const KDL::CoDyCo::FTSensorList * p_ft_list;
    
    bool verbose;
    
    bool consider_ft_offset;
    
    std::vector< std::string > subtree_leaf_links;
    std::vector< int > subtree_leaf_links_indeces;
    
    std::vector< int > subtree_links_indices; /** indeces of the links belonging to the considered subtree */
    
    const std::vector<int> linkIndeces2regrCols;
    
    int NrOfRealLinks_subtree;
    
    int isSubtreeLeaf(const int link_id) const;


    
    public:
        /**
         * Constructor for subtree articulated dynamics regressor 
         * 
         * @param _subtree_leaf_links the list of name of the leaf links of the considered subtree
         */
        subtreeArticulatedDynamicsRegressor(const KDL::CoDyCo::TreeGraph & _tree_graph, 
                                            const KDL::CoDyCo::FTSensorList & _ft_list, 
                                            const std::vector<int> & _linkIndeces2regrCols,
                                            std::vector< std::string> _subtree_leaf_links=std::vector< std::string>(0),
                                            const bool _consider_ft_offset=false,
                                            bool _verbose=true):
                                            p_tree_graph(&_tree_graph),
                                            p_ft_list(&_ft_list),
                                            linkIndeces2regrCols(_linkIndeces2regrCols),
                                            subtree_leaf_links(_subtree_leaf_links),
                                            consider_ft_offset(_consider_ft_offset),
                                            subtree_links_indices(0),
                                            verbose(_verbose),
                                            NrOfRealLinks_subtree(0)
        {
            assert(linkIndeces2regrCols.size() == p_tree_graph->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < linkIndeces2regrCols.size(); ll++ ) { if( linkIndeces2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= linkIndeces2regrCols.size());
        }
                                                                                                                                             
       ~subtreeArticulatedDynamicsRegressor() {};
        
        int getNrOfOutputs();
        
        int computeRegressor(const KDL::JntArray &q, 
                              const KDL::JntArray &q_dot, 
                              const KDL::JntArray &q_dotdot,
                              const std::vector<KDL::Frame> & X_dynamic_base,
                              const std::vector<KDL::Twist> &v,
                              const std::vector<KDL::Twist> & a,
                              const std::vector< KDL::Wrench > & measured_wrenches,
                              const KDL::JntArray & measured_torques,
                              Eigen::MatrixXd & regressor_matrix,
                              Eigen::VectorXd & known_terms);
        
        int configure();
        
    
};

}
#endif
