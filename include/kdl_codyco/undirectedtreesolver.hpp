/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef UNDIRECTED_TREE_SOLVER_H
#define UNDIRECTED_TREE_SOLVER_H

#include <kdl/tree.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include "kdl_codyco/treeserialization.hpp"

namespace KDL
{
namespace CoDyCo 
{
    /**
    * \brief This is the base class for all the Tree solvers (kinematics,
     *  dynamics, COM, ... ) that use the UndirectedTree object 
     * 
     *  
     */
    class UndirectedTreeSolver
    {
        protected: 
            const UndirectedTree undirected_tree;
            Traversal traversal;
            
        public:
            UndirectedTreeSolver(const Tree & tree_arg, const TreeSerialization & serialization_arg): 
                undirected_tree(tree_arg,serialization_arg)
            { undirected_tree.compute_traversal(traversal); assert(undirected_tree.check_consistency(traversal) == 0);
 };
         
            ~UndirectedTreeSolver() {};
            
            bool changeBase(std::string new_base_name)
            {
               int ret = undirected_tree.compute_traversal(traversal,new_base_name);
               if( ret != 0 ) { return false; }
               
               return true;            }
            
            bool changeBase(int new_base_id)
            {
                if( new_base_id < 0 || new_base_id >= undirected_tree.getNrOfLinks() ) {
                    return false;
                }
               
               int ret = undirected_tree.compute_traversal(traversal,new_base_id);
               if( ret != 0 ) { return false; }
               
               return true;
            }
            
            const TreeSerialization getSerialization() const
            {
                return undirected_tree.getSerialization();
            }
    };

}
}

#endif
