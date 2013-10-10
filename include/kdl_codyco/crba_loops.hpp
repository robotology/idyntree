/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_CRBA_LOOPS_HPP
#define KDL_CODYCO_CRBA_LOOPS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_codyco/undirectedtree.hpp>

#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl_codyco/floatingjntspaceinertiamatrix.hpp>

namespace KDL {
namespace CoDyCo {
   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the fixed base mass matrix
    * 
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    */
    int crba_fixed_base_loop(const UndirectedTree & undirected_tree, const Traversal & traversal, const JntArray & q, std::vector<RigidBodyInertia> & Ic, JntSpaceInertiaMatrix & H);
  
   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the floating base mass matrix
    * 
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    */
   int crba_floating_base_loop(const UndirectedTree & undirected_tree, const Traversal & traversal, const JntArray & q, std::vector<RigidBodyInertia> & Ic, FloatingJntSpaceInertiaMatrix & H);

}
}  



#endif //KDL_CODYCO_COM_LOOPS_HPP
