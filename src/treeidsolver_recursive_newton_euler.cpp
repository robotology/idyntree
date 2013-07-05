// Copyright (C)   2013 Silvio Traversaro - CoDyCo project http://www.codyco.eu/

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

///  This and the source file were modified from ChainIdSolver_recursive_newton_euler.h/cpp
///  Thus, still probably covered by LGPL, which is why I build it as a library and then link to it
///     Darren Earl, HRL 6/2012

//versione extracted from r2_controllers

#include "kdl_codyco/treeidsolver_recursive_newton_euler.hpp"
#include "kdl/frames_io.hpp"
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <deque>
using namespace std;

namespace KDL{
namespace CoDyCo {
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, const TreeSerialization & serialization) 
    : tree_graph(tree,serialization)
    {
        //Initializing gravitational acceleration (if any)
        ag=Twist::Zero();
        
        //allocate vectors
        v.resize(tree.getNrOfSegments());
        a.resize(tree.getNrOfSegments());
        f.resize(tree.getNrOfSegments());
                
        tree_graph.compute_traversal(traversal);
    }
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, Vector grav, const TreeSerialization & serialization)
    : tree_graph(tree,serialization)
    {
      
        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());
        
        //allocate vectors
        v.resize(tree.getNrOfSegments());
        a.resize(tree.getNrOfSegments());
        f.resize(tree.getNrOfSegments());
        
        tree_graph.compute_traversal(traversal);
    }
    
    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        Wrench dummy;
        return CartToJnt(q,q_dot,q_dotdot,Twist::Zero(),ag,f_ext,torques,dummy);
    }

    
    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext,JntArray &torques, Wrench& base_force)
    {
        assert(torques.rows() == tree_graph.getNrOfDOFs());
        
        base_force = Wrench::Zero();

        rneaKinematicLoop(tree_graph,q,q_dot,q_dotdot,traversal,base_velocity,base_acceleration,v,a);
        
        rneaDynamicLoop(tree_graph,q,traversal,v,a,f_ext,f,torques,base_force);
        
        return 0;
    }
    
    TreeSerialization TreeIdSolver_RNE::getSerialization() const 
    {
        return tree_graph.getSerialization();
    }

}
}//namespace
