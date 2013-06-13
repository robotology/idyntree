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

#include <deque>
using namespace std;

namespace KDL{
namespace CoDyCo {
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, const TreeSerialization & serialization) 
    : TreeSerialSolver(tree,serialization)
    {
        //duplicate TreeSerialSolver inizialitation !!, should fix 
        TreeIdSolver_RNE(tree,Vector::Zero(),serialization);
    }
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, Vector grav, const TreeSerialization & serialization)
    : TreeSerialSolver(tree,serialization)
    {
      
        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());
        
        //allocate vectors
        X.resize(tree.getNrOfSegments());
        v.resize(tree.getNrOfSegments());
        a.resize(tree.getNrOfSegments());
        f.resize(tree.getNrOfSegments());
        
        S.resize(tree.getNrOfJoints());
        
        //Get root name
		root_name = tree.getRootSegment()->first;

    }
    
    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        Wrench dummy;
        return CartToJnt(q,q_dot,q_dotdot,Twist::Zero(),ag,f_ext,torques,dummy);
    }

    
    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext,JntArray &torques, Wrench& base_force)
    {
        assert(torques.rows() == tree.getNrOfJoints());
        
        base_force = Wrench::Zero();
        
        int l;

        //rneaKinematicLoop(q,q_dot,q_dotdot,base_velocity,base_acceleration,recursion_order,index2segment,parent,link2joint,X,S,v,a,f);

        // process processed back to front...        
        //Sweep from leafs to root, recursion order in reverse
        for(l=recursion_order.size()-1; l >= 0; l--) {
            int curr_index = recursion_order[l];
            
			const Segment& seg = index2segment[curr_index]->second.segment;
            
			const Joint& jnt = seg.getJoint();
			Frame& eX = X[curr_index];
            Wrench& ef = f[curr_index];
            
            /*
            int parent_index = parent[curr_index];
            if( parent_index >= 0 ) {
                Entry& parent_e = db[parent[curr_index]];
                Wrench& pre_f = parent_e.f;
                pre_f +=eX*ef;
            } else {
                base_force += eX*ef;
            }

            if(link2joint[curr_index]!=FIXED_JOINT)
                torques(link2joint[curr_index])=dot(S[link2joint[curr_index]],ef);
            */
        }
        

        
        return 0;
    }

}
}//namespace
