/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/position_loops.hpp"

namespace KDL {
namespace CoDyCo {

    int getFrameLoop(const TreeGraph & tree_graph,
                const JntArray &q, 
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
                const JntArray &q, 
                const Traversal & traversal,
                const int proximal_link_index,
                const int distal_link_index)
    {
        Frame frame_proximal_distal;
        getFrameLoop(tree_graph,q,traversal,proximal_link_index,distal_link_index,frame_proximal_distal);
        return frame_proximal_distal;
    }
}
}
