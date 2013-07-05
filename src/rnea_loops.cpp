/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/rnea_loops.hpp"

namespace KDL {
namespace CoDyCo {

    int rneaKinematicLoop(const TreeGraph & tree_graph,
                           const JntArray &q, 
                           const JntArray &q_dot,
                           const JntArray &q_dotdot,  
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity, 
                           const Twist& base_acceleration, 
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a)
    {
        for(int i=0; i < (int)kinetic_traversal.order.size(); i++) {
            double joint_pos, joint_vel, joint_acc;
            LinkMap::const_iterator link_it = kinetic_traversal.order[i];
            int link_nmbr = link_it->link_nr; 
            if( i == 0 ) {
                assert( kinetic_traversal.parent[link_nmbr] == tree_graph.getInvalidLinkIterator() );
                v[link_nmbr] = base_velocity;
                a[link_nmbr] = base_acceleration;
            } else {
                LinkMap::const_iterator parent_it = kinetic_traversal.parent[link_it->link_nr];
                int parent_nmbr = parent_it->link_nr;
                
                if( link_it->getAdjacentJoint(parent_it)->joint.getType() != Joint::None ) {
                    int dof_nr = link_it->getAdjacentJoint(parent_it)->q_nr;
                    joint_pos = q(dof_nr);
                    joint_vel = q_dot(dof_nr);
                    joint_acc = q_dotdot(dof_nr);
                } else {
                    joint_pos = joint_vel = joint_acc = 0.0;
                }
                KDL::Frame X_son_parent = parent_it->pose(link_it,joint_pos);
                KDL::Twist S_son_parent = parent_it->S(link_it,joint_pos);
                KDL::Twist vj_son_parent = parent_it->vj(link_it,joint_pos,joint_vel);
                v[link_nmbr] = X_son_parent*v[parent_nmbr] + vj_son_parent;
                a[link_nmbr] = X_son_parent*a[parent_nmbr] + S_son_parent*joint_acc + v[link_nmbr]*vj_son_parent;
                
                #ifndef NDEBUG
                //std::cout << "v[ " << link_it->second.link_name << " ] = " << v[link_nmbr] << std::endl;
                //std::cout << "a[ " << link_it->second.link_name << " ] = " << a[link_nmbr] << std::endl;
                //std::cout << "\t=" << X_son_parent*a[parent_nmbr] << " + "<< S_son_parent*joint_acc << " + " << std::endl; 
                #endif 
                
                /*
                move this code here from dynamic_loop (decomment it) if you have to improve performance
                if( f.size() != 0 ) {
                    //Collect RigidBodyInertia and external forces
                    RigidBodyInertia Ii= link_it->second.I;
                    f[link_nmbr]=Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr])-f_ext[link_nmbr];
                }
                */
            }
        }
        return 0;
    }
    
    int rneaDynamicLoop(const TreeGraph & tree_graph,
                         const JntArray &q, 
                         const Traversal & dynamical_traversal,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         JntArray &torques,
                         Wrench & base_force)
    {
        double joint_pos;
        //move this loop back in kinetic loop to improve performance
        for(int l=dynamical_traversal.order.size()-1; l>=0; l-- ) {  
            LinkMap::const_iterator link_it = dynamical_traversal.order[l];
            int link_nmbr = link_it->link_nr;
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii= link_it->I;
            f[link_nmbr]=Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr])-f_ext[link_nmbr];
        }    
        //end loop to move (comment) to improve performance
          
        //Proper dynamic recursive loop for wrench calculations
        for(int l=dynamical_traversal.order.size()-1; l>=0; l-- ) {
            LinkMap::const_iterator link_it = dynamical_traversal.order[l];
            int link_nmbr = link_it->link_nr;
            
            if( l != 0 ) {
                
                LinkMap::const_iterator parent_it = dynamical_traversal.parent[link_nmbr];
                const int parent_nmbr = parent_it->link_nr;
                JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                
                if( joint_it->joint.getType() == Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->q_nr);
                }
                
                f[parent_nmbr] += link_it->pose(parent_it,joint_pos)*f[link_nmbr]; 
                #ifndef NDEBUG
                //std::cout << "f[ " << link_it->second.link_name << " ] = " << f[link_nmbr] << std::endl;
                #endif 
                
                if( joint_it->joint.getType() != Joint::None ) {
                    torques(link_it->getAdjacentJoint(parent_it)->q_nr) = dot(parent_it->S(link_it,joint_pos),f[link_nmbr]);
                }
                
            } else {
                
                assert( dynamical_traversal.parent[link_nmbr] == tree_graph.getInvalidLinkIterator() );
                
                base_force = f[link_nmbr];
            }
        }

        return 0;
        
    } 
    
}
}
