/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#ifndef TREE_COM_SOLVER_H
#define TREE_COM_SOLVER_H

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_codyco/treeserialization.hpp"

namespace KDL
{

    typedef std::vector<Wrench> Wrenches;

	/**
	 * \brief This  <strong>abstract</strong> class encapsulates the 
     *  solver for finding the Center Of Mass of a KDL::Tree 
	 */
	class TreeCOMSolver
	{
		public:
            explicit TreeCOMSolver(const Tree& tree, TreeSerialization serialization=TreeSerialization());

            virtual ~TreeCOMSolver();
            
			/** 
			 * Calculate the tree COM with respect to the base frame of reference
			 * 
			 * @param q input joint positions
             * 
			 * @return if < 0 something went wrong
			 */
            int JntToCOM(const JntArray& q_in, Vector& p_out);
        
        private:
            KDL::Tree tree;
            
            //vector containing the center of mass of each subtree 
            //subtree_COM[i] contains the center of mass of the subtree starting 
            //at link i (included)
            std::vector<KDL::Vector> subtree_COM;
            
            //vector containing the mass of each subtree 
            //subtree_mass[i] contains the mass of the subtree starting 
            //at link i (included)
            std::vector<double> subtree_mass;
            
            //serialization quantites
            std::vector< int> mu_root; //set of childrens of root
            std::vector< std::vector<int> > mu; //set of childrens of each segment
            std::vector< int > lambda; //parent of each segment
            std::vector< int> link2joint;
            std::vector< int > recursion_order;
            std::vector<SegmentMap::const_iterator> seg_vector;

	};

}

#endif
