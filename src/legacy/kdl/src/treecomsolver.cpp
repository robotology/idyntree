/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/treecomsolver.hpp>
#include <kdl_codyco/com_loops.hpp>

#include <iostream>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {

    TreeCOMSolver::TreeCOMSolver(const Tree& tree_in, TreeSerialization serialization_in):
                                UndirectedTreeSolver(tree_in,serialization_in),
                                subtree_first_moment_of_mass(undirected_tree.getNrOfLinks()),
                                subtree_mass(undirected_tree.getNrOfLinks())
    {
    }

    TreeCOMSolver::~TreeCOMSolver()
    {
    }

    int TreeCOMSolver::JntToCOM(const KDL::CoDyCo::GeneralizedJntPositions& q_in,
                                Vector& p_out)
    {
        //First we check all the sizes:
        if (q_in.jnt_pos.rows() != undirected_tree.getNrOfDOFs()) {
            return -1;
        }

        getCenterOfMassLoop(undirected_tree,q_in,traversal,subtree_first_moment_of_mass,subtree_mass,p_out);

        return 0;

    }

}
}
