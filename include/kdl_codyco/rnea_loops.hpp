/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_RNEA_LOOPS_HPP
#define KDL_CODYCO_RNEA_LOOPS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/treegraph.hpp>

namespace KDL {
namespace CoDyCo {

    /**
     * Perform the kinetic phase of the RNEA algorithm 
     * 
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     */
     int rneaKinematicLoop(const TreeGraph & tree_graph,
                           const JntArray &q, 
                           const JntArray &q_dot,
                           const JntArray &q_dotdot,  
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity, 
                           const Twist& base_acceleration, 
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a);
                       
    /**
     * Perform the dynamics phase of the RNEA algorithm, where the kinematic
     * quantites (kin_X) where calculate with a loop with a different base
     * 
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     * 
     */
    int rneaDynamicLoop(const TreeGraph & tree_graph,
                         const JntArray &q, 
                         const Traversal & dynamical_traversal,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         JntArray &torques,
                         Wrench & base_force);             
      
}
}  



#endif 
