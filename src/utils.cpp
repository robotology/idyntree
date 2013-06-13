/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/utils.hpp"
#include "kdl_codyco/treeserialization.hpp"

namespace KDL {
namespace CoDyCo {
    double computeMass(const Tree & tree) {
        
        double total_mass = 0.0;
        
        //create necessary vectors
        SegmentMap::const_iterator root;
        
        tree.getRootSegment(root);
        
        SegmentMap sm = tree.getSegments();
           
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            //root has no mass
            if( i != root ) {
               total_mass += i->second.segment.getInertia().getMass();
            }
		}
        
        return total_mass;
    }
    /*
    int rneaKinematicLoop(const JntArray &q, 
                      const JntArray &q_dot,
                      const JntArray &q_dotdot,  
                      const Twist& base_velocity, 
                      const Twist& base_acceleration, 
                      const std::vector<int>& visit_order,
                      const std::vector<SegmentMap::const_iterator>& index2segment,
                      const std::vector<int>& parent, 
                      const std::vector<int>& link2joint,
                              std::vector<Frame>& X,
                              std::vector<Twist>& S,
                              std::vector<Twist>& v,
                              std::vector<Twist>& a,
                              std::vector<Wrench>& f
                      )
    {
        //Sweep from root to leaf
        for( l = 0; l < visit_order.size(); l++ ) {
            
            unsigned int curr_index = visit_order[l];
        
			const Segment& seg = index2segment[curr_index]->second.segment;
			
            double q_,qdot_,qdotdot_;
            
            int idx = link2joint[curr_index];
            
            if( idx != FIXED_JOINT ) {
                q_=q(idx);
                qdot_=q_dot(idx);
                qdotdot_=q_dotdot(idx);
            } else {
                q_=qdot_=qdotdot_=0.0;
            }
                
            Frame& eX  = X[curr_index];
            Twist& ev  = v[curr_index];
            Twist& ea  = a[curr_index];

            //Calculate segment properties: X,S,vj,cj
            eX=seg.pose(q_);//Remark this is the inverse of the 
                            //frame for transformations from 
                            //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=eX.M.Inverse(seg.twist(q_,qdot_));
            S[link2joint[curr_index]]=eX.M.Inverse(seg.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            
            int parent_index = parent[curr_index];
            Twist parent_a, parent_v;
            
            if( parent_index == -1 ) {
                parent_a = base_acceleration;
                parent_v = base_velocity;
            } else {
                parent_a = a[parent_index];
                parent_v = v[parent_index];
            }
            
            ev=eX.Inverse(parent_v)+vj;
            ea=eX.Inverse(parent_a)+eS*qdotdot_+ev*vj;
            // end kinematic phase 
            
            if( f.size() != 0 ) {
                //Collect RigidBodyInertia and external forces
                RigidBodyInertia Ii=seg.getInertia();
                f[curr_index]=Ii*a[curr_index]+v[curr_index]*(Ii*v[curr_index])-f_ext[curr_index];
            }
            
            return 0;
        }
    }
    */
    /*
    {
        int l;
        for(l=visit_order.size()-1; l >= 0; l--) {
            int curr_index = visit_order[l];
            
			const Segment& seg = index2segment[curr_index]->second.segment;
                        
			Frame& eX = X[curr_index];
            Twist& eS = S[curr_index];
    
            int parent_index = parent[curr_index];
            if( parent_index >= 0 ) {
                f[parent_index] += project(f[curr_index],parent_index,curr_index,kinX,kin_parent);
            } else {
                base_force += project(f[curr_index],parent_index,curr_index,kinX,kin_parent);
            }

            if(link2joint[curr_index]!=FIXED_JOINT) {
                if( dyn_parent[curr_index] == kin_parent[curr_index] ) {
                    //If nothing for this joint has changed
                    torques(link2joint[curr_index])=dot(S(link2joint[curr_index],f[parent_index]);
                } else {
                    torques(link2joint[curr_index])=dot(project(S(link2joint[curr_index],f[current_index]);
                }
            }
        }
    }
    
    Vector project(const Vector & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent) 
    {
        #ifndef NDEBUG
        if( parent[i] != j && parent[j] != i ) return Vector::Zero();
        #endif
        if( parent(j) == i ) {
            return X[j]*arg;
        } else {
            //parent(i) == j
            return X[i].Inverse(arg);
        }
    }
    
    Wrench project(const Wrench & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent) 
    {
        #ifndef NDEBUG
        if( parent[i] != j && parent[j] != i ) return Wrench::Zero();
        #endif
        if( parent(j) == i ) {
            return X[j]*arg;
        } else {
            //parent(i) == j
            return X[i].Inverse(arg);
        }
    }
    
    Twist project(const Twist & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent) 
    {
        #ifndef NDEBUG
        if( parent[i] != j && parent[j] != i ) return Twist::Zero();
        #endif
        if( parent(j) == i ) {
            return X[j]*arg;
        } else {
            //parent(i) == j
            return X[i].Inverse(arg);
        }
    }
*/

}
}
