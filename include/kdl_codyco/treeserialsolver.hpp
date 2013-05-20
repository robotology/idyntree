/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef TREE_SERIAL_SOLVER_H
#define TREE_SERIAL_SOLVER_H

#include <kdl/tree.hpp>
#include "kdl_codyco/treeserialization.hpp"

namespace KDL
{
    /**
	 * \brief This is the base class for all the Tree solvers (kinematics,
     *  dynamics, COM, ... ) that use the TreeSerialization object 
	 *
	 */
	class TreeSerialSolver
	{
        protected: 
            Tree tree; /**< the local copy of the used KDL::Tree */
            TreeSerialization serialization; /**< the local copy of the TreeSerialization */
            
            std::vector<SegmentMap::const_iterator> index2segment; /**< vector that maps each link index to the iterator of the corresponding segment  */
            std::vector<int> childrens_root; /**< indices of the childrens of the root segment */
            std::vector< std::vector<int> > childrens; /**< indices of the childrens of each segment */
            std::vector< int > parent; /**< index of the parent of each segment */
            std::vector<int> link2joint; /**< vector that maps each link index to the iterator of the corresponding joint index, it the joint is not null, -1 otherwise */
            std::vector<int > recursion_order; /**< Vector that contains the indices of all the links,
                                                    ordered in way such that a link comes before that any
                                                    of it childrens, useful to emulate an order visit to 
                                                    each node of the tree */
                    
            
		public:
			TreeSerialSolver(const Tree & tree_arg, const TreeSerialization & serialization_arg);
            ~TreeSerialSolver();
            
            const TreeSerialization & getSerialization() const;


	};

}

#endif
