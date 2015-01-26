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

#include "kdl_codyco/chainidsolver_recursive_newton_euler_floating_base.hpp"
#include <kdl/frames_io.hpp>

namespace KDL{
    
    ChainIdSolver_RNE_FB::ChainIdSolver_RNE_FB(const Chain& chain_,Vector grav):
        chain(chain_),nj(chain.getNrOfJoints()),ns(chain.getNrOfSegments()),
        X(ns),S(ns),v(ns),a(ns),f(ns)
    {
        ag=-Twist(grav,Vector::Zero());
        assert(v.size() == ns);

    }
    
    int ChainIdSolver_RNE_FB::CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const Wrenches& f_ext, KDL::JntArray &torques)
    {
        Wrench dummy;
        return CartToJnt(q,q_dot,q_dotdot,Twist::Zero(),ag,f_ext,torques,dummy);
    }



    int ChainIdSolver_RNE_FB::CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot,  const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext, KDL::JntArray &torques, Wrench& base_force)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
            return -1;
        unsigned int j=0;

        //Sweep from root to leaf
        for(unsigned int i=0;i<ns;i++){
            double q_,qdot_,qdotdot_;
            Segment segm;
            segm = chain.getSegment(i);
            if(segm.getJoint().getType()!=Joint::None){
                q_=q(j);
                qdot_=q_dot(j);
                qdotdot_=q_dotdot(j);
                j++;
            }else
                q_=qdot_=qdotdot_=0.0;
            
            //Calculate segment properties: X,S,vj,cj
            X[i]=segm.pose(q_);//Remark this is the inverse of the
                                                //frame for transformations from 
                                                //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=X[i].M.Inverse(segm.twist(q_,qdot_));
            S[i]=X[i].M.Inverse(segm.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            if(i==0){
                v[i]=X[i].Inverse(base_velocity)+vj;
                a[i]=X[i].Inverse(base_acceleration)+S[i]*qdotdot_+v[i]*vj;
            }else{
                v[i]=X[i].Inverse(v[i-1])+vj;
                a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;
            }
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii=segm.getInertia();
            f[i]=Ii*a[i]+v[i]*(Ii*v[i])-f_ext[i];
            //std::cout << "aLink " << segm.getName() << "\na= " << a[i]  << "\nv= " << v[i] << "\nf= " << f[i] << "\nf_ext= " << f_ext[i]  << std::endl;
            //std::cout << "a["<<i<<"]=" << a[i] << "\n f["<<i<<"]=" << f[i] << "\n S["<<i<<"]=" << S[i] << std::endl;
        }
        //Sweep from leaf to root
        j=nj-1;
        for(int i=ns-1;i>=0;i--){
        	Segment segm;
        	segm = chain.getSegment(i);
            if(segm.getJoint().getType()!=Joint::None)
                torques(j--)=dot(S[i],f[i]);
            if(i!=0)
                f[i-1]=f[i-1]+X[i]*f[i];
        }
        base_force = X[0]*f[0];
        //debug
        for(int i=0; i < (int)ns; i++) {
            Segment segm;
        	segm = chain.getSegment(i);
            //std::cout << "bLink " << segm.getName() << " a= " << a[i] << " f= " << f[i] << std::endl;
        }
        
	return 0;
    }
}//namespace
