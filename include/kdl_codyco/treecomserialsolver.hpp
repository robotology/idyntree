/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#ifndef TREE_COM_SERIALSOLVER_H
#define TREE_COM_SERIALSOLVER_H

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "kdl_codyco/treeserialization.hpp"
#include "kdl_codyco/treeserialsolver.hpp"

namespace KDL
{
namespace CoDyCo
{
	/**
	 * \brief This  <strong>abstract</strong> class encapsulates the 
     *  solver for finding the Center Of Mass of a KDL::Tree 
	 */
	class TreeCOMSerialSolver : TreeSerialSolver 
	{
		public:
            explicit TreeCOMSerialSolver(const Tree& tree,const TreeSerialization & serialization=TreeSerialization());

            virtual ~TreeCOMSerialSolver();
            
			/** 
			 * Calculate the tree COM with respect to the base frame of reference
			 * 
			 * @param q input joint positions
             * 
			 * @return if < 0 something went wrong
			 */
            int JntToCOM(const JntArray& q_in, Vector& p_out);
        
        private:
            
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
