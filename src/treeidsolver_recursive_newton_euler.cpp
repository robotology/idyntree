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
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree_,Vector grav,TreeSerialization serialization)
    :tree(tree_)
    {
        //should compile only in debug mode
        if(!serialization.is_consistent(tree)) {
            serialization = TreeSerialization(tree);
        }
        
        //Get root name
		root_name = tree.getRootSegment()->first;
		
        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());
        
        //the deprecated method is more efficient
        const SegmentMap& sm = tree.getSegments();
        
        //allocate vectors
        db.resize(tree.getNrOfSegments());
        mu_root.resize(0);
        mu.resize(tree.getNrOfSegments(),std::vector<unsigned int>(0));
        lambda.resize(tree.getNrOfSegments());
        
        link2joint.resize(tree.getNrOfSegments(),tree.getNrOfSegments());
                
        seg_vector.resize(tree.getNrOfSegments());
        
        
        //create necessary vectors
        SegmentMap::const_iterator root, i;
        
        tree.getRootSegment(root);
        for( unsigned int j=0; j < root->second.children.size(); j++ ) {
            mu_root.push_back(serialization.getLinkId(root->second.children[j]->first));
        }
        
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            if( i != root ) {
                unsigned int i_index = serialization.getLinkId(i->first);
                seg_vector[i_index] = i;
                
                for( unsigned int j=0; j < i->second.children.size(); j++ ) {
                    mu[i_index].push_back(serialization.getLinkId(i->second.children[j]->first));
                }
                
                if( i->second.segment.getJoint().getType() != Joint::None ) {
                    link2joint[i_index] = serialization.getJointId(i->first);
                }
                
                if( i->second.parent == root ) {
                    lambda[i_index] = -1;
                } else {
                    lambda[i_index] = serialization.getLinkId(i->second.parent->first);
                }
                
            }
		}
        
        //As the order of the recursion is the same, it is calculated only at configuration
        std::vector<unsigned int> index_stack;
        
        index_stack.reserve(tree.getNrOfSegments());
        recursion_order.reserve(tree.getNrOfSegments());
        
        index_stack.clear();
        recursion_order.clear();
        
        for( unsigned int j=0; j < mu_root.size(); j++ ) {
            index_stack.push_back(mu_root[j]);
        }
        
        while( !index_stack.empty() ) {
            
            unsigned int curr_index = index_stack.back();
            index_stack.pop_back();
            
            recursion_order.push_back(curr_index);
            
            //Doing the recursion on the children
            for( unsigned int j=0; j < mu[curr_index].size(); j++ ) {
                index_stack.push_back(mu[curr_index][j]);
            }
        }
        
        assert(recursion_order.size() == tree.getNrOfSegments());
        
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

        //Sweep from root to leaf
        for( l = 0; l < recursion_order.size(); l++ ) {
            
            unsigned int curr_index = recursion_order[l];
        
			const Segment& seg = seg_vector[curr_index]->second.segment;
			const Joint& jnt = seg.getJoint();
			
            double q_,qdot_,qdotdot_;
            if(jnt.getType() !=Joint::None){
				int idx = link2joint[curr_index];
                q_=q(idx);
                qdot_=q_dot(idx);
                qdotdot_=q_dotdot(idx);
            }else
                q_=qdot_=qdotdot_=0.0;
                
            Entry& e = db[curr_index];
                
            Frame& eX  = e.X;
            Twist& eS  = e.S;
            Twist& ev  = e.v;
            Twist& ea  = e.a;
            Wrench& ef = e.f;
                        

            //Calculate segment properties: X,S,vj,cj
            eX=seg.pose(q_);//Remark this is the inverse of the 
                            //frame for transformations from 
                            //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=eX.M.Inverse(seg.twist(q_,qdot_));
            eS=eX.M.Inverse(seg.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            
            int parent_index = lambda[curr_index];
            Twist parent_a, parent_v;
            
            if( parent_index == -1 ) {
                parent_a = base_acceleration;
                parent_v = base_velocity;
            } else {
                Entry& parent_entry = db[parent_index];
            
                parent_a = parent_entry.a;
                parent_v = parent_entry.v;
            }
            
            ev=eX.Inverse(parent_v)+vj;
            ea=eX.Inverse(parent_a)+eS*qdotdot_+ev*vj;
            
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii=seg.getInertia();
            ef=Ii*ea+ev*(Ii*ev)-f_ext[curr_index];
            //std::cout << "aLink " << seg.getName() << "\na= " << ea << "\nv= " << ev << "\nf= " << ef  << "\nf_ext= " << ef_ext << std::endl;
        }
        
        // process processed back to front...        
        //Sweep from leafs to root, recursion order in reverse
        for(l=recursion_order.size()-1; l >= 0; l--) {
            unsigned int curr_index = recursion_order[l];
            
			const Segment& seg = seg_vector[curr_index]->second.segment;
			const Joint& jnt = seg.getJoint();
			Entry& e = db[curr_index];
			Frame& eX = e.X;
            Twist& eS = e.S;
            Wrench& ef = e.f;
    
            int parent_index = lambda[curr_index];
            if( parent_index >= 0 ) {
                Entry& parent_e = db[lambda[curr_index]];
                Wrench& pre_f = parent_e.f;
                pre_f +=eX*ef;
            } else {
                base_force += eX*ef;
            }

            if(jnt.getType()!=Joint::None)
                torques(link2joint[curr_index])=dot(eS,ef);
        }
        

        //debug
        //for( map<string, Entry>::const_iterator i= db.begin(); i!= db.end(); ++i ){
            //std::cout << "bLink " << i->first << " a= " << i->second.a << " f= " << i->second.f << std::endl;
		//}
        
        return 0;
    }


}//namespace
