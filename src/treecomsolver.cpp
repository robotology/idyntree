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

    TreeCOMSolver::TreeCOMSolver(const Tree& tree_in, TreeSerialization serialization_in): undirected_tree(tree_in,serialization_in)
    {
        subtree_COM.resize(undirected_tree.getNrOfLinks());
        subtree_mass.resize(undirected_tree.getNrOfLinks());
        #ifndef NDEBUG
        std::cerr << "Check consistency in the constructor of TreeComSolver" << std::endl;
        //assert(.check_consistency() == 0);
        #endif
        //Using default base
        int ret = undirected_tree.compute_traversal(traversal);
        assert( ret==0 );
        //Avoiding unused variable warning
        ((void)ret);
    }

    TreeCOMSolver::~TreeCOMSolver() {
    }

    int TreeCOMSolver::JntToCOM(const KDL::JntArray& q_in, Vector& p_out) {
        //First we check all the sizes:
        if (q_in.rows() != undirected_tree.getNrOfDOFs()) {
            return -1;
        }

        getCenterOfMassLoop(undirected_tree,q_in,traversal,subtree_COM,subtree_mass,p_out);

        return 0;

    }

}
}
