// Copyright  (C)  2007 Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse
// Copyright  (C)  2013 Silvio Traversaro

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "kdl_codyco/treefksolverpos_iterative.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {
    TreeFkSolverPos_iterative::TreeFkSolverPos_iterative(const Tree& tree_arg,
                                                         const std::string & base_link, 
                                                         TreeSerialization serialization_arg):    
                tree_graph(tree_arg,serialization_arg)
    {

        #ifndef NDEBUG
        //std::cerr << "Check consistency in the constructor of TreeFkSolverPos" << std::endl;
        //assert(tree_graph.check_consistency() == 0);
        #endif
        
        int ret = tree_graph.compute_traversal(traversal,base_link);
        assert(tree_graph.check_consistency(traversal) == 0);

        if( ret != 0 ) {
            tree_graph.compute_traversal(traversal);
            assert(tree_graph.check_consistency(traversal) == 0);

        }
    }
    
    TreeFkSolverPos_iterative::~TreeFkSolverPos_iterative()
    {
    }
    
    int TreeFkSolverPos_iterative::setBaseLink(const std::string & base_link)
    {
        assert(tree_graph.check_consistency(traversal) == 0);
        int ret = tree_graph.compute_traversal(traversal,base_link);
        assert(tree_graph.check_consistency(traversal) == 0);
        if( ret != 0 ) {
            //If error, restore default state
            tree_graph.compute_traversal(traversal);
            assert(tree_graph.check_consistency(traversal) == 0);
            return -1;
        } else {
            return 0;
        }
    }

    int TreeFkSolverPos_iterative::JntToCart(const JntArray& q_in, Frame& p_out, std::string segmentName)
    {
        assert(tree_graph.check_consistency(traversal) == 0);
        
        if( q_in.rows() != tree_graph.getNrOfDOFs() )
            return -1;
            
        LinkMap::const_iterator it;
        it = tree_graph.getLink(segmentName);
        
        if( it == tree_graph.getInvalidLinkIterator() ) 
            return -2;
        
        Frame currentFrame, resultFrame;
        resultFrame = Frame::Identity();
        
        for(LinkMap::const_iterator link=it; link != traversal.order[0]; link = traversal.parent[link->link_nr] ) {
            LinkMap::const_iterator parent_link = traversal.parent[link->link_nr];
            assert( parent_link != tree_graph.getInvalidLinkIterator() );
            
            double joint_position;
            
            if( link->getAdjacentJoint(parent_link)->joint.getType() != Joint::None ) {
                joint_position = q_in((link->getAdjacentJoint(parent_link))->q_nr);
            } else {
                joint_position =0;
            }
            
            currentFrame = link->pose(parent_link,
                                             joint_position);
                                             
            resultFrame = currentFrame*resultFrame;
        }
        
        p_out = resultFrame;
        return 0;
    }

    int TreeFkSolverPos_iterative::JntToCart(const JntArray& q_in, Frame& p_out, int segmentIndex)
    {
        if( q_in.rows() != tree_graph.getNrOfDOFs() )
            return -1;
        
        LinkMap::const_iterator it;
        
        
        return 0;        	
    }
}
    


}
