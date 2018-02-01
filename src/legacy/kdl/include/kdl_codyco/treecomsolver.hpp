/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef TREE_COM_SOLVER_H
#define TREE_COM_SOLVER_H

#ifdef __DEPRECATED
  #warning <treecomsolver.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "treeserialization.hpp"
#include "undirectedtree.hpp"
#include "undirectedtreesolver.hpp"
#include "generalizedjntpositions.hpp"

namespace KDL
{
namespace CoDyCo
{

        /**
          * \brief This  class encapsulates the
          *  solver for finding the Center Of Mass of a KDL::Tree
          */
        class TreeCOMSolver : public UndirectedTreeSolver
        {
        public:
            explicit TreeCOMSolver(const Tree& tree,TreeSerialization serialization=TreeSerialization());

            virtual ~TreeCOMSolver();

            /**
             * Calculate the tree COM with respect to the world frame of reference
             *
             * @param q_in input generalized joint positions
             *
             * @param p_out COM in the base link reference frame
             *
             * @return if < 0 something went wrong
             */
            int JntToCOM(const KDL::CoDyCo::GeneralizedJntPositions & q_in, Vector& p_out);

        private:
            //vector containing the center of mass of each subtree
            //subtree_COM[i] contains the center of mass of the subtree starting
            //at link i (included)
            std::vector<KDL::Vector> subtree_first_moment_of_mass;

            //vector containing the mass of each subtree
            //subtree_mass[i] contains the mass of the subtree starting
            //at link i (included)
            std::vector<double> subtree_mass;


	};
}
}
#endif
