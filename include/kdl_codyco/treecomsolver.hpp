/**
 * Copyright  (C) 2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#ifndef TREE_COM_SOLVER_H
#define TREE_COM_SOLVER_H

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "kdl_codyco/treeserialization.hpp"
#include "kdl_codyco/undirectedtree.hpp"

namespace KDL
{
namespace CoDyCo
{

	/**
	 * \brief This  <strong>abstract</strong> class encapsulates the 
     *  solver for finding the Center Of Mass of a KDL::Tree 
	 */
	class TreeCOMSolver 
	{
        public:
            explicit TreeCOMSolver(const Tree& tree,TreeSerialization serialization=TreeSerialization(),TreePartition partition=TreePartition());

            virtual ~TreeCOMSolver();
            
            /** 
             * Calculate the tree COM with respect to the base frame of reference
             * 
             * @param q_in input joint positions
             * 
             * @param p_out COM in the base link reference frame
             * @param part_id the id of the part for which the COM has to be calculated, -1 for all the body
             * 
             * @return if < 0 something went wrong
             */
            int JntToCOM(const KDL::JntArray& q_in, Vector& p_out, const int part_id=-1);
        
        private:
            const UndirectedTree undirected_tree;
            Traversal traversal;
            
            //vector containing the center of mass of each subtree 
            //subtree_COM[i] contains the center of mass of the subtree starting 
            //at link i (included)
            std::vector<KDL::Vector> subtree_COM;
            
            //vector containing the mass of each subtree 
            //subtree_mass[i] contains the mass of the subtree starting 
            //at link i (included)
            std::vector<double> subtree_mass;
            

	};
}
}
#endif
