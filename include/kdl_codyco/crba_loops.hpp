/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_CRBA_LOOPS_HPP
#define KDL_CODYCO_CRBA_LOOPS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include "kdl_codyco/undirectedtree.hpp"

#include "kdl_codyco/floatingjntspaceinertiamatrix.hpp"
#include "kdl_codyco/momentumjacobian.hpp"

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

   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the jacobian of the momentum expressed with the orientation of the base link 
    * and with respect to the center of mass. The algorithm used is a modified 
    * version of the CRBA, as explained in:
     * @article{Orin2013,
     *      author = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
     *      doi = {10.1007/s10514-013-9341-4},
     *      issn = {0929-5593},
     *      journal = {Autonomous Robots},
     *      title = {{Centroidal dynamics of a humanoid robot}},
     *      volume = {35},
     *      year = {2013}
     * }
     *  In this paper this momentum jacobian is called "Centroidal Momentum Matrix"
    * 
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    */
   int crba_momentum_jacobian_loop(const UndirectedTree & undirected_tree, 
                                    const Traversal & traversal, 
                                    const JntArray & q, 
                                    std::vector<RigidBodyInertia> & Ic, 
                                    MomentumJacobian & H,
                                    RigidBodyInertia & InertiaCOM
                                   );
}
}  



#endif //KDL_CODYCO_COM_LOOPS_HPP
