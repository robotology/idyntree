/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/rnea_loops.hpp"

namespace KDL {
namespace CoDyCo {

    int rneaKinematicLoop(const UndirectedTree & undirected_tree,
                           const KDL::JntArray &q, 
                           const KDL::JntArray &q_dot,
                           const KDL::JntArray &q_dotdot,  
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity, 
                           const Twist& base_acceleration, 
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a)
    {
        for(int i=0; i < (int)kinetic_traversal.getNrOfVisitedLinks(); i++) {
            double joint_pos, joint_vel, joint_acc;
            LinkMap::const_iterator link_it = kinetic_traversal.getOrderedLink(i);
            int link_nmbr = link_it->getLinkIndex(); 
            if( i == 0 ) {
                assert( kinetic_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
                v[link_nmbr] = base_velocity;
                a[link_nmbr] = base_acceleration;
            } else {
                LinkMap::const_iterator parent_it = kinetic_traversal.getParentLink(link_it);
                int parent_nmbr = parent_it->getLinkIndex();
                
                if( link_it->getAdjacentJoint(parent_it)->getJoint().getType() != Joint::None ) {
                    int dof_nr = link_it->getAdjacentJoint(parent_it)->getDOFIndex();
                    joint_pos = q(dof_nr);
                    joint_vel = q_dot(dof_nr);
                    joint_acc = q_dotdot(dof_nr);
                } else {
                    joint_pos = joint_vel = joint_acc = 0.0;
                }
                KDL::Frame X_child_parent = parent_it->pose(link_it,joint_pos);
                KDL::Twist S_child_parent = parent_it->S(link_it,joint_pos);
                //KDL::Twist vj_child_parent = parent_it->vj(link_it,joint_pos,joint_vel);
                KDL::Twist vj_child_parent = S_child_parent*joint_vel;
                v[link_nmbr] = X_child_parent*v[parent_nmbr] + vj_child_parent;
                a[link_nmbr] = X_child_parent*a[parent_nmbr] + S_child_parent*joint_acc + v[link_nmbr]*vj_child_parent;
                
                #ifndef NDEBUG
                //std::cout << "v[ " << link_it->second.link_name << " ] = " << v[link_nmbr] << std::endl;
                //std::cout << "a[ " << link_it->second.link_name << " ] = " << a[link_nmbr] << std::endl;
                //std::cout << "\t=" << X_child_parent*a[parent_nmbr] << " + "<< S_child_parent*joint_acc << " + " << std::endl; 
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
    
     int rneaKinematicLoop(const UndirectedTree & undirected_tree,
                           const KDL::JntArray &q, 
                           const KDL::JntArray &q_dot,
                           const KDL::JntArray &q_dotdot,  
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity, 
                           const Twist& base_acceleration, 
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a,
                                 std::vector<Wrench>& f_gi
                          )
    {
        for(int i=0; i < (int)kinetic_traversal.getNrOfVisitedLinks(); i++) {
            double joint_pos, joint_vel, joint_acc;
            LinkMap::const_iterator link_it = kinetic_traversal.getOrderedLink(i);
            int link_nmbr = link_it->getLinkIndex(); 
            if( i == 0 ) {
                assert( kinetic_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
                v[link_nmbr] = base_velocity;
                a[link_nmbr] = base_acceleration;
            } else {
                LinkMap::const_iterator parent_it = kinetic_traversal.getParentLink(link_it);
                int parent_nmbr = parent_it->getLinkIndex();
                
                if( link_it->getAdjacentJoint(parent_it)->getJoint().getType() != Joint::None ) {
                    int dof_nr = link_it->getAdjacentJoint(parent_it)->getDOFIndex();
                    joint_pos = q(dof_nr);
                    joint_vel = q_dot(dof_nr);
                    joint_acc = q_dotdot(dof_nr);
                } else {
                    joint_pos = joint_vel = joint_acc = 0.0;
                }
                KDL::Frame X_child_parent = parent_it->pose(link_it,joint_pos);
                KDL::Twist S_child_parent = parent_it->S(link_it,joint_pos);
                //KDL::Twist vj_child_parent = parent_it->vj(link_it,joint_pos,joint_vel);
                KDL::Twist vj_child_parent = S_child_parent*joint_vel;
                v[link_nmbr] = X_child_parent*v[parent_nmbr] + vj_child_parent;
                a[link_nmbr] = X_child_parent*a[parent_nmbr] + S_child_parent*joint_acc + v[link_nmbr]*vj_child_parent;
                
                #ifndef NDEBUG
                //std::cout << "v[ " << link_it->second.link_name << " ] = " << v[link_nmbr] << std::endl;
                //std::cout << "a[ " << link_it->second.link_name << " ] = " << a[link_nmbr] << std::endl;
                //std::cout << "\t=" << X_child_parent*a[parent_nmbr] << " + "<< S_child_parent*joint_acc << " + " << std::endl; 
                #endif 
            
            }
            
            if( f_gi.size() != 0 ) {
                    //Collect RigidBodyInertia and external forces
                    RigidBodyInertia Ii = link_it->getInertia();
                    f_gi[link_nmbr]=Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr]);
            }
        }
        return 0;
    }
    
    
    int rneaDynamicLoop(const UndirectedTree & undirected_tree,
                         const KDL::JntArray &q, 
                         const Traversal & dynamical_traversal,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         KDL::JntArray &torques,
                         Wrench & base_force)
    {
        double joint_pos;
        //move this loop back in kinetic loop to improve performance
        for(int l=dynamical_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {  
            LinkMap::const_iterator link_it = dynamical_traversal.getOrderedLink(l);
            int link_nmbr = link_it->getLinkIndex();
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii= link_it->getInertia();
            #ifdef NDEBUG
            f[link_nmbr]=Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr])-f_ext[link_nmbr];
            #else
            f[link_nmbr] = Ii*a[link_nmbr];
            f[link_nmbr] = f[link_nmbr] + v[link_nmbr]*(Ii*v[link_nmbr]);
            f[link_nmbr] = f[link_nmbr] - f_ext[link_nmbr];
            #endif
        }    
        //end loop to move (comment) to improve performance
          
        //Proper dynamic recursive loop for wrench calculations
        for(int l=dynamical_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            LinkMap::const_iterator link_it = dynamical_traversal.getOrderedLink(l);
            int link_nmbr = link_it->getLinkIndex();
            
            if( l != 0 ) {
                
                LinkMap::const_iterator parent_it = dynamical_traversal.getParentLink(link_nmbr);
                const int parent_nmbr = parent_it->getLinkIndex();
                JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                
                if( joint_it->getJoint().getType() == Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->getDOFIndex());
                }
                
                f[parent_nmbr] += link_it->pose(parent_it,joint_pos)*f[link_nmbr]; 
                #ifndef NDEBUG
                //std::cout << "f[ " << link_it->second.link_name << " ] = " << f[link_nmbr] << std::endl;
                #endif 
                
                if( joint_it->getJoint().getType() != Joint::None ) {
                    torques(link_it->getAdjacentJoint(parent_it)->getDOFIndex()) = dot(parent_it->S(link_it,joint_pos),f[link_nmbr]);
                }
                
            } else {
                
                assert( dynamical_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
                
                base_force = f[link_nmbr];
            }
        }

        return 0;
        
    } 
    
    
    
     int rneaDynamicLoop(const UndirectedTree & undirected_tree,
                         const KDL::JntArray &q, 
                         const Traversal & dynamical_traversal,
                         const std::vector<Wrench>& f_gi,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         KDL::JntArray &torques,
                         Wrench & base_force)
    {
        double joint_pos;
        assert(f.size() == f_gi.size());
        assert(f.size() == f_ext.size());
        
        for(int i=0; i < (int)dynamical_traversal.getNrOfVisitedLinks(); i++) {
            LinkMap::const_iterator link_it = dynamical_traversal.getOrderedLink(i);
            int link_nmbr = link_it->getLinkIndex();
            f[link_nmbr] = f_gi[link_nmbr] - f_ext[link_nmbr];
        }
        
        //Proper dynamic recursive loop for wrench calculations
        for(int l=dynamical_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            LinkMap::const_iterator link_it = dynamical_traversal.getOrderedLink(l);
            int link_nmbr = link_it->getLinkIndex();
            
            if( l != 0 ) {
                
                LinkMap::const_iterator parent_it = dynamical_traversal.getParentLink(link_nmbr);
                const int parent_nmbr = parent_it->getLinkIndex();
                JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                
                if( joint_it->getJoint().getType() == Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->getDOFIndex());
                }
                
                f[parent_nmbr] += link_it->pose(parent_it,joint_pos)*f[link_nmbr]; 
                #ifndef NDEBUG
                //std::cout << "f[ " << link_it->second.link_name << " ] = " << f[link_nmbr] << std::endl;
                #endif 
                
                if( joint_it->getJoint().getType() != Joint::None ) {
                    torques(link_it->getAdjacentJoint(parent_it)->getDOFIndex()) = dot(parent_it->S(link_it,joint_pos),f[link_nmbr]);
                }
                
            } else {
                
                assert( dynamical_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
                
                base_force = f[link_nmbr];
            }
        }

        return 0;
        
    } 
    
    
}
}
