/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
  
#include "torqueRegressor.hpp"

#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>

using namespace KDL::CoDyCo;

namespace KDL {
namespace CoDyCo {
namespace Regressors {
    
bool torqueRegressor::isActiveFTSensor(const int ft_sensor_id) const
{
    if( activated_ft_sensors.size() != (unsigned int)p_ft_list->getNrOfFTSensors() ) { return false; }
    if( ft_sensor_id < 0 || ft_sensor_id >= p_ft_list->getNrOfFTSensors() ) { return false; }
    return activated_ft_sensors[ft_sensor_id];
}

int torqueRegressor::configure()
{      
    const KDL::CoDyCo::UndirectedTree & undirected_tree = *p_undirected_tree;
    const KDL::CoDyCo::FTSensorList & ft_list = *p_ft_list;
   
    
    //Now store all the links that belong to the subtree
    //We first compute the spanning starting at one (the first) of the leafs 
    KDL::CoDyCo::Traversal subtree_traversal;
    
    
    JunctionMap::const_iterator torque_jnt = undirected_tree.getJunction(torque_dof);
    if( torque_jnt == undirected_tree.getInvalidJunctionIterator() ) { 
        if(verbose) { std::cerr << "torqueRegressor error: specified joint " << torque_dof << " not found " << std::endl; }
        return -1; 
    }
    
    torque_dof_index = torque_jnt->getJunctionIndex();
    
    relative_junction.resize(1);
    relative_junction[0] = torque_dof_index;
    
    
    if( torque_dof_index < 0 || torque_dof_index >= (int)undirected_tree.getNrOfDOFs() ) {
         if(verbose) { std::cerr << "torqueRegressor error: specified joint " << torque_dof << " has no degrees of freedom " << std::endl; }
        return -1; 
    }
    
    
    int link_parent_id = torque_jnt->getParentLink()->getLinkIndex();
    int link_child_id = torque_jnt->getChildLink()->getLinkIndex();
    
    if( !reverse_direction ) {
        subtree_root_link_id = link_child_id;
    } else {
        subtree_root_link_id = link_parent_id;
    }
    
    undirected_tree.compute_traversal(subtree_traversal,subtree_root_link_id);
    
    //Then we color the nodes: if white (true) the links belongs to the subtree
    //if black (false) it does not belongs to the node
    std::vector<bool> is_link_in_subtree(undirected_tree.getNrOfLinks(),false);
    
    //the leaf that we chose as starting link clearly belongs to the subtree
    is_link_in_subtree[subtree_root_link_id] = true;
    
    //then, all the other links can be classified using the following rules:  
    for(int i=1; i < (int)subtree_traversal.getNrOfVisitedLinks(); i++ ) {
        int link_id = subtree_traversal.getOrderedLink(i)->getLinkIndex();
        int parent_id = subtree_traversal.getParentLink(link_id)->getLinkIndex();
        
        if( is_link_in_subtree[parent_id] == false ) {
            //if the parent is not in the subtree (false/black) then the link is not in the subtree (false/black)
            is_link_in_subtree[link_id] = false;
        } else {
            int junction_index = undirected_tree.getLink(link_id)->getAdjacentJoint(undirected_tree.getLink(parent_id))->getJunctionIndex();
            
            if( ft_list.isFTSensor(junction_index) && isActiveFTSensor(ft_list.getFTSensorID(junction_index)) ) {
                //if the parent is in the subtree (true/white) but it is connected to the link by an activated FT sensor
                is_link_in_subtree[link_id] = false;
                //In this case we have to add the parent to subtree_leaf_links_indeces
                subtree_leaf_links_indeces.push_back(parent_id);
                
            } else if ( junction_index == torque_dof_index ) {
                //or if the parent is connected trought the torque regressor dof
                is_link_in_subtree[link_id] = false;
            } else {
                //otherwise the link is in the subtree (true/white)
                is_link_in_subtree[link_id] = true;
            }
        }
    }
    
    //after all the links belonging to the subtree have been identified, it is possible to save only the id of the one in the subtree
    assert( subtree_links_indices.size() == 0 );
    for( int i=0; i < (int)is_link_in_subtree.size(); i++ ) {
        if( is_link_in_subtree[i] ) {
            subtree_links_indices.push_back(i);
        }
    }
   
    
    
    return 0;
}

int torqueRegressor::getNrOfOutputs()
{
    return 1;
}

std::vector<int> torqueRegressor::getRelativeJunctions()
{
    return relative_junction;
}

int torqueRegressor::computeRegressor(const KDL::JntArray &q, 
                                      const KDL::JntArray &/*q_dot*/,
                                      const KDL::JntArray &/*q_dotdot*/,
                                      const std::vector<KDL::Frame> & X_dynamic_base,
                                      const std::vector<KDL::Twist> & v,
                                      const std::vector<KDL::Twist> & a,
                                      const std::vector< KDL::Wrench > & measured_wrenches,
                                      const KDL::JntArray & measured_torques,
                                      Eigen::MatrixXd & regressor_matrix,
                                      Eigen::VectorXd & known_terms)
{
#ifndef NDEBUG
    if( verbose ) std::cerr << "Called torqueRegressor::computeRegressor " << std::endl;
#endif 
    //const KDL::CoDyCo::UndirectedTree &  = *p_undirected_tree;
    const KDL::CoDyCo::FTSensorList & ft_list = *p_ft_list;

    
    if( regressor_matrix.rows() != getNrOfOutputs() ) {
        return -1;
    }
    
         /**
         * \todo move this stuff in UndirectedTree
         * 
         */
        JunctionMap::const_iterator torque_dof_it = p_undirected_tree->getJunction(torque_dof_index);
        LinkMap::const_iterator parent_root_it;
        if( torque_dof_it->getChildLink() == p_undirected_tree->getLink(subtree_root_link_id) ) {
            parent_root_it = torque_dof_it->getParentLink();
        } else {
            parent_root_it = torque_dof_it->getChildLink();
        }
        assert(torque_dof_it->getJunctionIndex() < (int)p_undirected_tree->getNrOfDOFs());
        KDL::Twist S = parent_root_it->S(p_undirected_tree->getLink(subtree_root_link_id),q(torque_dof_it->getJunctionIndex()));
        
    //all other columns, beside the one relative to the inertial parameters of the links of the subtree, are zero
    regressor_matrix.setZero();
    
    for(int i =0; i < (int)subtree_links_indices.size(); i++ ) {
        int link_id = subtree_links_indices[i];
        
        #ifndef NDEBUG
        if( verbose ) std::cerr << "Adding to the torque regressor of joint " << torque_dof_it->getName() << " the regressor relative to link " << p_undirected_tree->getLink(link_id)->getName() << std::endl;
        #endif
            
        if( linkIndeces2regrCols[link_id] != -1 ) {
            Eigen::Matrix<double,6,10> netWrenchRegressor_i = netWrenchRegressor(v[link_id],a[link_id]);
            regressor_matrix.block(0,(int)(10*linkIndeces2regrCols[link_id]),getNrOfOutputs(),10) = toEigen(S).transpose()*WrenchTransformationMatrix(X_dynamic_base[subtree_root_link_id].Inverse()*X_dynamic_base[link_id])*netWrenchRegressor_i;
        }        
    }
    
#ifndef NDEBUG
    if( consider_ft_offset ) {
        if( verbose ) std::cerr << "considering ft offset" << std::endl;
    } else {
        if( verbose ) std::cerr << "not considering ft offset" << std::endl;
    }
#endif
    //if the ft offset is condidered, we have to set also the columns relative to the offset 
    //For each subgraph, we consider the measured wrenches as the one excerted from the remain of the tree to the considered subtree 
    //So the sign of the offset should be consistent with the other place where it is defined. In particular, the offset of a sensor is considered
    //as an addictive on the measured wrench, so f_s = f_sr + f_o, where the frame of expression and the sign of f_s is defined in 
    //KDL::CoDyCo::FTSensor
    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ ) {
        if( consider_ft_offset ) {
            int leaf_link_id = subtree_leaf_links_indeces[leaf_id];
            
            std::vector< FTSensor> fts_link = ft_list.getFTSensorsOnLink(leaf_link_id);
             
            assert(fts_link.size()==1);
            
            int ft_id = fts_link[0].getID();
            assert(fts_link[0].getChild() == leaf_link_id || fts_link[0].getParent() == leaf_link_id );
            
            #ifndef NDEBUG
            if( verbose ) std::cerr << "Adding to the torque regressor of joint " << torque_dof_it->getName() << " the columns relative to offset of ft sensor " << fts_link[0].getName() << std::endl;
            #endif
            
            /** \todo find a more robust way to get columns indeces relative to a given parameters */
            assert(ft_id >= 0 && ft_id < 100);
            assert(10*NrOfRealLinks_subtree+6*ft_id+5 < regressor_matrix.cols());
            
            double sign;
            if( fts_link[0].isWrenchAppliedFromParentToChild() ) {
                sign = 1.0;
            } else {
                sign = -1.0;
            }
            
            if( fts_link[0].getParent() == leaf_link_id ) { 
                regressor_matrix.block(0,(int)(10*NrOfRealLinks_subtree+6*ft_id),getNrOfOutputs(),6) = -sign*toEigen(S).transpose()*WrenchTransformationMatrix(X_dynamic_base[subtree_root_link_id].Inverse()*X_dynamic_base[leaf_link_id]*fts_link[0].getH_link_sensor(leaf_link_id));
            } else {
                assert( fts_link[0].getChild() == leaf_link_id );
                regressor_matrix.block(0,(int)(10*NrOfRealLinks_subtree+6*ft_id),getNrOfOutputs(),6) = sign*toEigen(S).transpose()*WrenchTransformationMatrix(X_dynamic_base[subtree_root_link_id].Inverse()*X_dynamic_base[leaf_link_id]*fts_link[0].getH_link_sensor(leaf_link_id));
            }
            
        }
    }
    
    //The known terms are simply all the measured wrenches acting on the subtree projected on the axis and plus the measured torque
    known_terms(0) = measured_torques(torque_dof_index);
    
    #ifndef NDEBUG
    //std::cerr << "computing kt " << std::endl;
    //std::cerr << (ft_list).toString();
#endif 
    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ ) {
        int leaf_link_id = subtree_leaf_links_indeces[leaf_id];
            
        std::vector< FTSensor> fts_link = ft_list.getFTSensorsOnLink(leaf_link_id);
             
        assert(fts_link.size()==1);
            
        //int ft_id = fts_link[0].getID();
#ifndef NDEBUG
        //std::cerr << "For leaf " << leaf_link_id << " found ft sensor " << ft_id << " that connects " << fts_link[0].getParent() << " and " << fts_link[0].getChild() << std::endl;
#endif
        
        assert(fts_link[0].getChild() == leaf_link_id || fts_link[0].getParent() == leaf_link_id );


        known_terms(0) +=  dot(S,X_dynamic_base[subtree_root_link_id].Inverse()*X_dynamic_base[leaf_link_id]*ft_list.getMeasuredWrench(leaf_link_id,measured_wrenches));
    }
    
#ifndef NDEBUG
/*
std::cout << "Returning from computeRegressor (subtree):" << std::endl;
std::cout << "Regressor " << std::endl;
std::cout << regressor_matrix << std::endl;
std::cout << "known terms" << std::endl;
std::cout << known_terms << std::endl;
*/
#endif
    
    return 0;
}

}

}

}
