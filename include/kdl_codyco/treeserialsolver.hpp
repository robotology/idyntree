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
     * The rationale behind the use this base class, that creates an 
     * index based representation of the KDL::Tree of the solver, is to
     * interface in a efficient way with serial data structures that
     *  are given as an input to the solver (for example, joint positions/
     * speeds/acceleration or link external forces. 
     * In the existing KDL::Tree data structure, and serialization structure
     * for the joint is embedded, however there is a lack of serialization
     * for the links. 
     * And index base structure (parent and children vectors) is also created
     * because, if for navigating the KDL::Tree pointer structure is used,
     * then for each new visited segment a map between the segment and the link
     * index would be necessary, and it was preferred to avoid the use of maps.
     * 
     * 
     *    
	 *
	 */
	class TreeSerialSolver
	{
        protected: 
            Tree tree; /**< the local copy of the used KDL::Tree */
            TreeSerialization serialization; /**< the local copy of the TreeSerialization */
            
            std::vector<SegmentMap::const_iterator> index2segment; /**< vector that maps each link index to the iterator of the corresponding segment  */
            std::vector<int> children_root; /**< indices of the children of the root segment */
            std::vector< std::vector<int> > children; /**< indices of the children of each segment */
            std::vector< int > parent; /**< index of the parent of each segment */
            std::vector<int> link2joint; /**< vector that maps each link index to the iterator of the corresponding joint index, it the joint is not null, -1 otherwise */
            std::vector<int > recursion_order; /**< Vector that contains the indices of all the links,
                                                    ordered in way such that a link comes before that any
                                                    of it children, useful to emulate an order visit to 
                                                    each node of the tree */
                    
            
		public:
			TreeSerialSolver(const Tree & tree_arg, const TreeSerialization & serialization_arg);
            ~TreeSerialSolver();
            
            const TreeSerialization & getSerialization() const;


	};

}

#endif
