// Copyright  (C)  2007 Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse
/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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
