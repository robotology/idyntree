// Copyright  (C)  2013  Silvio Traversaro <silvio dot traversaro at iit dot it >
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

#ifndef KDLCODYCOTREEDYNPARAM_HPP
#define KDLCODYCOTREEDYNPARAM_HPP

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/articulatedbodyinertia.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <kdl_codyco/floatingjntspaceinertiamatrix.hpp>
#include <kdl_codyco/undirectedtreesolver.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>

namespace KDL {
namespace CoDyCo {

    /**
     * Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) 
     * for the calculation torques out of the pose and derivatives.
     * (inverse dynamics)
     *
     * The algorithm implementation for H is based on the book "Rigid Body
     * Dynamics Algorithms" of Roy Featherstone, 2008
     * (ISBN:978-0-387-74314-1) See page 107 for the pseudo-code.
     * This algorithm is extended for the use of fixed joints
     *
     * It calculates the joint-space inertia matrix, given the motion of
     * the joints (q,qdot,qdotdot), external forces on the segments
     * (expressed in the segments reference frame) and the dynamical
     * parameters of the segments.
     */
    class TreeDynParam: public UndirectedTreeSolver
    {
    public:
        TreeDynParam(const Tree& chain, Vector _grav=Vector::Zero(), const TreeSerialization & _serialization=TreeSerialization());
        ~TreeDynParam();

        virtual int JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis);
        virtual int JntToMass(const JntArray &q, JntSpaceInertiaMatrix& H); 
        virtual int JntToGravity(const JntArray &q,JntArray &gravity);
        
        virtual int JntToCoriolis(const JntArray &q, const JntArray &q_dot, const Twist &base_vel,  JntArray &coriolis_gravity_torques, Wrench & coriolis_gravity_base_wrench);
        virtual int JntToMass(const JntArray &q, FloatingJntSpaceInertiaMatrix& H); 

    private:
        int nr;
        unsigned int nj;
        unsigned int ns;	
        Vector grav;
        Vector vectornull;
        JntArray jntarraynull;
        TreeIdSolver_RNE treeidsolver_coriolis;
        TreeIdSolver_RNE treeidsolver_gravity;
        std::vector<Wrench> wrenchnull;
        std::vector<Frame> X;
        std::vector<Twist> S;
        std::vector<RigidBodyInertia> Ic;
        Wrench F;
        Twist ag;
	
    };

}
}
#endif
