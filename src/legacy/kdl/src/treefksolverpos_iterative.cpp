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

#include <kdl_codyco/treefksolverpos_iterative.hpp>
#include <kdl_codyco/position_loops.hpp>

#include <iostream>
#include <kdl/frames_io.hpp>


namespace KDL {
namespace CoDyCo {
    TreeFkSolverPos_iterative::TreeFkSolverPos_iterative(const Tree& tree_arg,
                                                         TreeSerialization serialization_arg):
                UndirectedTreeSolver(tree_arg,serialization_arg)
    {
    }

    TreeFkSolverPos_iterative::~TreeFkSolverPos_iterative()
    {
    }


    int TreeFkSolverPos_iterative::JntToCart(const KDL::CoDyCo::GeneralizedJntPositions& q_in,
                                             Frame& p_out, std::string segmentName)
    {
        LinkMap::const_iterator it;
        it = undirected_tree.getLink(segmentName);

        if( it == undirected_tree.getInvalidLinkIterator() )
            return -2;

        return JntToCart(q_in,p_out,it->getLinkIndex());
    }

    int TreeFkSolverPos_iterative::JntToCart(const KDL::CoDyCo::GeneralizedJntPositions& q_in,
                                             Frame& p_out, int segmentIndex)
    {
        assert(undirected_tree.check_consistency(traversal) == 0);

        if( q_in.jnt_pos.rows() != undirected_tree.getNrOfDOFs() )
            return -1;

        LinkMap::const_iterator it;
        it = undirected_tree.getLink(segmentIndex);

        if( it == undirected_tree.getInvalidLinkIterator() )
            return -2;

        getWorldFrameLoop(undirected_tree,q_in,traversal,segmentIndex,p_out);


        return 0;
    }
}



}
