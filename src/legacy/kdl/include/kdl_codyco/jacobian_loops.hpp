/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_JACOBIAN_LOOPS_HPP
#define KDL_CODYCO_JACOBIAN_LOOPS_HPP

#ifdef __DEPRECATED
  #warning <jacobian_loops.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include "undirectedtree.hpp"
#include "generalizedjntpositions.hpp"

namespace KDL {
namespace CoDyCo {

   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the
    * fixed base (relative) jacobian for a given link
    * @param jac a 6 x nrOfDOFs Jacobian such that \f$ v_link_index = J dq + v_traversal_base \f$ expressed in link_index frame
    */
   void getRelativeJacobianLoop(const UndirectedTree & ,
                                const KDL::JntArray &q,
                                const Traversal & traversal,
                                const int link_index,
                                Jacobian & jac);
   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the
    * floating base jacobian for a given link with respect to the base defined in the Traversal
    * @param jac a 6 x (nrOfDOFs+6) Jacobian such that \f$ v_link_index = J dq_fl \f$ expressed in the frame of the link with ID link_index frame
    */
   void getFloatingBaseJacobianLoop(const UndirectedTree & ,
                                   const KDL::JntArray &q,
                                   const Traversal & traversal,
                                   const int link_index,
                                   Jacobian & jac);
   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the
    * floating base jacobian for a given link with respect to the base defined in the Traversal
    * @param jac a 6 x (nrOfDOFs+6) Jacobian such that \f$ v_link_index = J dq_fl \f$ expressed in the frame of the link with ID link_index frame
    * Where v_link_index is the 6d velocity of the selected link with respect to the link origin, but expressed in world orientation
    */
   void getFloatingBaseJacobianLoop(const UndirectedTree & ,
                                    const KDL::CoDyCo::GeneralizedJntPositions &q,
                                    const Traversal & traversal,
                                    const int link_index,
                                    Jacobian & jac);
}
}



#endif
