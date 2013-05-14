// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

//versione extracted from r2_controllers


#pragma once

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

namespace KDL
{

    typedef std::vector<Wrench> Wrenches;

	/**
	 * \brief This  class encapsulates the routines
     *  for calculating the current Center Of Mass of a KDL::Tree 
	 */
	class TreeIdSolver
	{
		public:
			/** 
			 * Calculate inverse dynamics, from joint positions, velocity, acceleration, external forces
			 * to joint torques/forces.
			 * 
			 * @param q input joint positions
			 * @param q_dot input joint velocities
			 * @param q_dotdot input joint accelerations
			 *
			 * @param torque output joint torques
			 * 
			 * @return if < 0 something went wrong
			 */
        virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)=0;
        
        
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
			 * @param torque output joint torques
			 * @param base_wrench output base wrench
             * 
			 * @return if < 0 something went wrong
			 */
        virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext,JntArray &torques, Wrench& base_force)=0;



        // Need functions to return the manipulator mass, coriolis and gravity matrices - Lagrangian Formulation.
	};

}
