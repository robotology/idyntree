// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse
// Copyright  (C)  2013 Silvio Traversaro

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

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

#ifndef KDLTREEFKSOLVERPOS_ITERATIVE_HPP
#define KDLTREEFKSOLVERPOS_ITERATIVE_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/treeserialization.hpp>
#include "kdl_codyco/treeserialsolver.hpp"
#include "kdl_codyco/treegraph.hpp"

namespace KDL {
namespace CoDyCo {

    /**
     * Implementation of a iterative forward position kinematics
     * algorithm to calculate the position transformation from joint
     * space to Cartesian space of a general kinematic tree (KDL::Tree),
     * using TreeSerialization.
     * 
     * \todo add copy constructor?: also for other solvers ? 
     *
     */
    class TreeFkSolverPos_iterative
    {
    private:
        const TreeGraph tree_graph;
        Traversal traversal;
    
    public:
        TreeFkSolverPos_iterative (const Tree& tree_arg, const std::string & base_link="", TreeSerialization serialization_arg=TreeSerialization());
        ~TreeFkSolverPos_iterative();
        
        /**
         * Change the base_link of the solver
         * 
         */
        int setBaseLink(const std::string & base_link);
        
        TreeGraph getTreeGraph() const { return tree_graph; }

        int JntToCart(const KDL::JntArray& q_in, Frame& p_out, std::string segmentName);
        int JntToCart(const KDL::JntArray& q_in, Frame& p_out, const int segmentIndex);
        
    };

}
}

#endif
