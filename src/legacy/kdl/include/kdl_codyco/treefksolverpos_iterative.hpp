// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse
// Copyright  (C)  2013 Silvio Traversaro

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDLTREEFKSOLVERPOS_ITERATIVE_HPP
#define KDLTREEFKSOLVERPOS_ITERATIVE_HPP

#ifdef __DEPRECATED
  #warning <treefksolverpos_iterative.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "treeserialization.hpp"
#include "undirectedtree.hpp"
#include "undirectedtreesolver.hpp"
#include "generalizedjntpositions.hpp"

namespace KDL {
namespace CoDyCo {

    /**
     * Implementation of a iterative forward position kinematics
     * algorithm to calculate the position transformation from joint
     * space to Cartesian space of a general kinematic tree (KDL::Tree).
     *
     * \todo add copy constructor?: also for other solvers ?
     *
     */
    class TreeFkSolverPos_iterative: public UndirectedTreeSolver
    {
    private:

    public:
        TreeFkSolverPos_iterative (const Tree& tree_arg, TreeSerialization serialization_arg=TreeSerialization());
        ~TreeFkSolverPos_iterative();

        int JntToCart(const KDL::CoDyCo::GeneralizedJntPositions& q_in,
                      Frame& p_out, std::string segmentName);
        int JntToCart(const KDL::CoDyCo::GeneralizedJntPositions& q_in,
                      Frame& p_out, const int segmentIndex);

    };

}
}

#endif
