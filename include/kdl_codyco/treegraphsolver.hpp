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
namespace CoDyCo 
{
    /**
	 * \brief This is the base class for all the Tree solvers (kinematics,
     *  dynamics, COM, ... ) that use the TreeGraph object 
     * 
     *  
	 */
	class TreeGraphSolver
	{
        protected: 
            const TreeGraph tree_graph;
            Traversal traversal;
            
		public:
			TreeGraphSolver(const Tree & tree_arg, const TreeSerialization & serialization_arg);
            ~TreeGraphSolver();
            
            bool changeBase(std::string new_base);
            
            const TreeSerialization & getSerialization() const;
	};

}
}

#endif
