/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_COM_LOOPS_HPP
#define KDL_CODYCO_COM_LOOPS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/momentumjacobian.hpp>

namespace KDL {
namespace CoDyCo {
   /**
    * Loop for calculating, given a TreeGraph and a Traversal, the center
    * of mass of the tree, expressed in frame of the root link of the Traversal
    * 
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    */
   void getCenterOfMassLoop(const TreeGraph & tree_graph,
                            const KDL::JntArray &q, 
                            const Traversal & traversal,
                            std::vector<KDL::Vector>& subtree_COM,
                            std::vector<double>& subtree_mass,
                            Vector & com,
                            int part_id = -1);
    
    
   /**
    * Loop for calculating, given a TreeGraph and a Traversal, the floating 
    * base jacobian (with the same assumptions of getAbsoluteJacobian)
    * of the momentum expressed in the frame of the base link of the Traversal,
    * divided by the overall mass of the tree. The first 
    * three rows of this jacobian are the exactly the jacobian of the center
    * of mass.
    * 
    * \warning Basic function designed for use inside the solver,so some the
    *          error checking on input/output parameters is not guaranteed
    * 
    * @param jac the 6x(NrOfDOFs+6) MomentumJacobian, used for the Momentum Jacobian output
    * @param buffer_1 a 6x(NrOfDOFs+6) Jacobian, used for intermediate results
    * @param buffer_2 a 6x(NrOfDOFs+6) MomentumJacobian, used for intermediate results
    * @param the total inertia of the tree, expressed in the base reference frame (useful to convert between the momentum jacobian and the COM jacobian)
    */
   void getMomentumJacobianLoop(const TreeGraph & tree_graph,
                                const KDL::JntArray &q, 
                                const Traversal & traversal,
                                const std::vector<Frame>& X_b,
                                MomentumJacobian & jac,
                                Jacobian & buffer_1,
                                MomentumJacobian & buffer_2,
                                RigidBodyInertia & total_inertia,
                                int part_id = -1);
   
   void getCOMJacobianLoop(const TreeGraph & tree_graph,
                           const KDL::JntArray &q, 
                           const Traversal & traversal,
                           const std::vector<Frame>& X_b,
                           Jacobian & jac,
                           Jacobian & buffer_jac,
                           int part_id = -1);
}
}  



#endif //KDL_CODYCO_COM_LOOPS_HPP
