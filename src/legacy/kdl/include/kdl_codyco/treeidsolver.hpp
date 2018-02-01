// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

//versione extracted from r2_controllers

#ifndef KDL_CODYCO_TREE_ID_SOLVER 
#define KDL_CODYCO_TREE_ID_SOLVER

#ifdef __DEPRECATED
  #warning <treeidsolver.hpp> is deprecated.
#endif

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

namespace KDL
{
namespace CoDyCo 
{
    typedef std::vector<Wrench> Wrenches;

    /**
     * \brief This  class encapsulates the routines
     *  for calculating the inverse dynamics of a KDL::Tree
     */
    class TreeIdSolver
    {
        public:
        
        
        virtual ~TreeIdSolver() {}
            /** 
             * Calculate inverse dynamics, from joint positions, velocity, acceleration, external forces
             * to joint torques/forces.
             * 
             * @param q input joint positions
             * @param q_dot input joint velocities
             * @param q_dotdot input joint accelerations
             *
             * @param f_ext external forces
             * @param torques output joint torques
             * 
             * @return if < 0 something went wrong
             */
        virtual int CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)=0;
        
        
        /** 
             * Calculate floating base inverse dynamics, from joint positions, velocity, acceleration, 
             * base velocity, base acceleration, external forces
             * to joint torques/forces.
             * 
             * @param q input joint positions
             * @param q_dot input joint velocities
             * @param q_dotdot input joint accelerations
             * @param base_velocity velocity of the floating base 
             *        (the linear part has no influence on the dynamics)
             * @param base_acceleration acceleration of the floating base 
             *        (proper acceleration, considers also gravitational acceleration)
             * @param f_ext external forces
             *
             * @param torques output joint torques
             * @param base_force output base wrench
             * 
             * @return if < 0 something went wrong
             */
        virtual int CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext,JntArray &torques, Wrench& base_force)=0;



        // Need functions to return the manipulator mass, coriolis and gravity matrices - Lagrangian Formulation.
    };

}
}


#endif
