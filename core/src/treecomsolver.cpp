/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "kdl_codyco/treecomsolver.hpp"
#include "kdl_codyco/com_loops.hpp"
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
