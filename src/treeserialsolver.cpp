/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/treeserialsolver.hpp"

namespace KDL {
    
    TreeSerialSolver::TreeSerialSolver(const Tree& tree_arg, const TreeSerialization & serialization_arg) : tree(tree_arg),
                                                                                                            serialization(serialization_arg)
    {
        if(!serialization.is_consistent(tree)) {
            serialization = TreeSerialization(tree);
        }
        
        serialization.serialize(tree,childrens_root,childrens,parent,link2joint,recursion_order,index2segment);

    }
    
    TreeSerialSolver::~TreeSerialSolver()
    {
    }
    
    const TreeSerialization & TreeSerialSolver::getSerialization() const
    {
        return serialization;
    }
    
    
}
