/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/jacobian_loops.hpp"
#include "kdl_codyco/regressor_utils.hpp"

#ifndef NDEBUG
#include <iostream>
#endif

namespace KDL {
namespace CoDyCo {
    
    void getRelativeJacobianLoop(const TreeGraph & tree_graph,
                                const KDL::JntArray &q, 
                                const Traversal & traversal,
                                const int link_index,
                                Jacobian & jac)
    {
        Frame T_total = Frame::Identity(); //The transformation between link_index frame and current_link frame
        
        KDL::CoDyCo::LinkMap::const_iterator current_link;
        current_link = tree_graph.getLink(link_index);
        
        //All the columns not modified are zero
        SetToZero(jac);
        
        KDL::CoDyCo::LinkMap::const_iterator parent_link=traversal.parent[current_link->getLinkIndex()];
        while(current_link != traversal.order[0]) {
            double joint_pos = 0.0;
            if( current_link->getAdjacentJoint(parent_link)->getNrOfDOFs() == 1 ) {
                KDL::Twist jac_col;
                
                int dof_index = current_link->getAdjacentJoint(parent_link)->getDOFIndex();
                
                joint_pos = q(dof_index);
                KDL::Twist S_current_parent = parent_link->S(current_link,joint_pos); 
                
                jac_col = T_total*S_current_parent;
                
                jac.setColumn(dof_index,jac_col);
            }
            
            KDL::Frame X_current_parent = parent_link->pose(current_link,joint_pos);
            
            T_total = T_total*X_current_parent;
            
            current_link = parent_link;
            parent_link = traversal.parent[current_link->getLinkIndex()];
        }

    }
    
    void getFloatingBaseJacobianLoop(const TreeGraph & tree_graph,
                                     const KDL::JntArray &q, 
                                     const Traversal & traversal,
                                     const int link_index,
                                     Jacobian & jac)
    {
        Frame T_total = Frame::Identity(); //The transformation between link_index frame and current_link frame
        
        assert(link_index < tree_graph.getNrOfLinks());
        
        KDL::CoDyCo::LinkMap::const_iterator current_link;
        current_link = tree_graph.getLink(link_index);
        
        //All the columns not modified are zero
        SetToZero(jac);
        
        KDL::CoDyCo::LinkMap::const_iterator parent_link=traversal.parent[current_link->getLinkIndex()];
        while(current_link != traversal.order[0]) {
            double joint_pos = 0.0;
            if( current_link->getAdjacentJoint(parent_link)->getNrOfDOFs() == 1 ) {
                KDL::Twist jac_col;
                
                int dof_index = current_link->getAdjacentJoint(parent_link)->getDOFIndex();
                
                joint_pos = q(dof_index);
                KDL::Twist S_current_parent = parent_link->S(current_link,joint_pos); 
                
                jac_col = T_total*S_current_parent;
                
                assert(6+dof_index < jac.columns());
                assert( dof_index < tree_graph.getNrOfDOFs() );
                jac.setColumn(6+dof_index,jac_col);
            }
            
            KDL::Frame X_current_parent = parent_link->pose(current_link,joint_pos);
            
            T_total = T_total*X_current_parent;
            
            current_link = parent_link;
            parent_link = traversal.parent[current_link->getLinkIndex()];
        }
        
        //Setting the floating part of the Jacobian
        jac.data.block(0,0,6,6) = TwistTransformationMatrix(T_total);
    }
    
}
}
