/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/position_loops.hpp"

namespace KDL {
namespace CoDyCo {

    int getFrameLoop(const TreeGraph & tree_graph,
                const KDL::JntArray &q, 
                const Traversal & traversal,
                const int proximal_link_index,
                const int distal_link_index,
                Frame & frame_proximal_distal)
    {
        LinkMap::const_iterator distal_it = tree_graph.getLink(distal_link_index);
        LinkMap::const_iterator proximal_it = tree_graph.getLink(proximal_link_index);
        
        Frame currentFrame;
        Frame resultFrame = Frame::Identity();
        for(LinkMap::const_iterator link=distal_it; link != proximal_it; link = traversal.parent[link->link_nr] ) {
            LinkMap::const_iterator parent_link = traversal.parent[link->link_nr];
            assert( parent_link != tree_graph.getInvalidLinkIterator() );
            
            double joint_position;
            
            if( link->getAdjacentJoint(parent_link)->joint.getType() != Joint::None ) {
                joint_position = q((link->getAdjacentJoint(parent_link))->q_nr);
            } else {
                joint_position =0;
            }
            
            currentFrame = link->pose(parent_link,
                                             joint_position);
                                             
            resultFrame = currentFrame*resultFrame;
        }
        
        frame_proximal_distal = resultFrame;
    
        return 0;
    }
    
    Frame getFrameLoop(const TreeGraph & tree_graph,
                const KDL::JntArray &q, 
                const Traversal & traversal,
                const int proximal_link_index,
                const int distal_link_index)
    {
        Frame frame_proximal_distal;
        getFrameLoop(tree_graph,q,traversal,proximal_link_index,distal_link_index,frame_proximal_distal);
        return frame_proximal_distal;
    }
    
    
    int getFramesLoop(const TreeGraph & tree_graph,
					  const KDL::JntArray &q, 
					  const Traversal & traversal,
					  std::vector<Frame> & X_base)
    {
          for(int i=0; i < (int)traversal.order.size(); i++) {
            double joint_pos;
            LinkMap::const_iterator link_it = traversal.order[i];
            int link_nmbr = link_it->link_nr; 
            if( i == 0 ) {
                assert( traversal.parent[link_nmbr] == tree_graph.getInvalidLinkIterator() );
                X_base[link_nmbr] = KDL::Frame::Identity();
            } else {
                LinkMap::const_iterator parent_it = traversal.parent[link_it->link_nr];
                int parent_nmbr = parent_it->link_nr;
                
                if( link_it->getAdjacentJoint(parent_it)->joint.getType() != Joint::None ) {
                    int dof_nr = link_it->getAdjacentJoint(parent_it)->q_nr;
                    joint_pos = q(dof_nr);
                } else {
                    joint_pos =  0.0;
                }
                KDL::Frame X_parent_son = link_it->pose(parent_it,joint_pos);
                X_base[link_nmbr] = X_base[parent_nmbr]*X_parent_son;
            }
        }
        return 0;
    }
}
}
