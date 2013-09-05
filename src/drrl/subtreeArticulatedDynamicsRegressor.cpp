/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
  
#include <drrl/subtreeArticulatedDynamicsRegressor.hpp> 

namespace DRRL
{

virtual int subtreeArticulatedDynamicsRegressor::configure(const KDL::CoDyCo::TreeGraph & tree_graph, const KDL::CoDyCo::FTSensorList & ft_list)
{
    //Checking if the provided subtree leafs define a proper subtree
    subtree_leaf_links_indeces.resize(subtree_leaf_links)
    
    for(int i=0; i < subtree_leaf_links.size(); i++ ) {
        LinkMap::const_iterator link_it = tree_graph.getLink(subtree_leaf_links[i]);
        
        if( link_it == tree_graph.getInvalidLinkIterator() ) {
            if( verbose ) { std::cerr << "subtreeArticulatedDynamicsRegressor::configure error: link " << subtree_leaf_links[i] << " not found " << std::endl; }
            return -1;
        }
        
        if( ft_list.getNrOfFTSensorsOnLink(link_it->getLinkIndex()) == 0 ) {
            if( verbose ) { std::cerr << "subtreeArticulatedDynamicsRegressor::configure error: link " << subtree_leaf_links[i] << " passed as a subtree leaf, but no FT sensor is attached to it " << std::endl; }
            return -2;
        }
        
        ///\todo add check that the indicated subtree is a proper one
        //ask to Daniele Pucci
        subtree_leaf_links_indices[i] = link_it->getLinkIndex(); 
    }
}

virtual int compute_regressor(const KDL::CoDyCo::TreeGraph & tree_graph,
                              const JntArray &q, 
                              const JntArray &q_dot, 
                              const JntArray &q_dotdot,
                              const std::vector<KDL::Frame> & X_dynamic_base,
                              const std::vector<KDL::Twist> v,
                              const std::vector< KDL::Wrench > & measured_wrenches;
                              const KDL::JntArray measured_torques,
                              Eigen::MatrixXd & regressor_matrix,
                              Eigen::VectorXd & known_terms)
{
    
}

}
