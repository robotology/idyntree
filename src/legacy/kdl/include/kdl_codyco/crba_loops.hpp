/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_CRBA_LOOPS_HPP
#define KDL_CODYCO_CRBA_LOOPS_HPP

#ifdef __DEPRECATED
  #warning <crba_loops.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include "undirectedtree.hpp"
#include "floatingjntspaceinertiamatrix.hpp"
#include "momentumjacobian.hpp"
#include "generalizedjntpositions.hpp"

namespace KDL {
namespace CoDyCo {
   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the fixed base mass matrix
    *
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    *
    * \param[in] undirected_tree the UndirectedTree model of the robot
    * \param[in] traversal       the traversal (containg the information about the used base link)
    * \param[in] q               vector (size: getNrOfDOFs() ) joint positions
    * \param[out] Ic             vector (size: getNrOfDOFs() ) temporary buffer
    *                            Ic[i] contains the composite rigid body inertia seen at joint i
    * \param[out] H              matrix (size: getNrOfDOFs() X getNrOfDOFs) containg the joint space Mass matrix
    */
    int crba_fixed_base_loop(const UndirectedTree & undirected_tree,
                             const Traversal & traversal,
                             const JntArray & q,
                             std::vector<RigidBodyInertia> & Ic,
                             JntSpaceInertiaMatrix & H);

   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the floating base mass matrix
    *
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    * \warning DEPRECATED.. do not use, use instead GeneralizedJntPositions
    */
   int crba_floating_base_loop(const UndirectedTree & undirected_tree,
                               const Traversal & traversal,
                               const JntArray & q,
                               std::vector<RigidBodyInertia> & Ic,
                               FloatingJntSpaceInertiaMatrix & H);

   /**
    * Loop for calculating, given a UndirectedTree, a Traversal and the joint position,
    * the (6+n_dof)x(6+n_dof) floating base mass matrix \f[ \mathbf{M} \f], such that the kinematic energy
    * of the system is given by:
    * \f[
    *  \dot{\mathbf{q}}^\top \mathbf{M} \dot{\mathbf{q}}
    * \f]
    * where \f[ \dot{\mathbf{q}} \in \mathbb{R}^{6+n} \f] is defined by abuse of notation as the concatenation of
    * \f[ {}^w \mathbf{v} \in \mathbb{R}^6 \f] (the floating base origin velocity expressed in world frame) and
    * \f[ {}^w \dot{\boldsymbol\theta} \in \mathbb{R}^n \f] (the joint velocity vector)
    *
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    */
   int crba_floating_base_loop(const UndirectedTree & undirected_tree,
                               const Traversal & traversal,
                               const GeneralizedJntPositions & q,
                               std::vector<RigidBodyInertia> & Ic,
                               FloatingJntSpaceInertiaMatrix & H);

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
