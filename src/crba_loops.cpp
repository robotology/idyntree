/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/crba_loops.hpp"

#include <kdl/kinfam.hpp>

#include <kdl_codyco/regressor_utils.hpp>
#include "kdl_codyco/undirectedtree.hpp"

#ifndef NDEBUG
#include <iostream>
#endif

#include <Eigen/Core>

namespace KDL {
namespace CoDyCo {

    int crba_fixed_base_loop(const UndirectedTree & undirected_tree, const Traversal & traversal, const JntArray & q, std::vector<RigidBodyInertia> & Ic, JntSpaceInertiaMatrix & H) {
        double q_;
        Wrench F;
        
        //Sweep from root to leaf
        for(int i=0;i<(int)traversal.getNrOfVisitedLinks();i++)
        {
          LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
          int link_index = link_it->getLinkIndex();
          
          //Collect RigidBodyInertia
          Ic[link_index] = link_it->getInertia();
        
        }
        
        for(int i=(int)traversal.getNrOfVisitedLinks()-1; i >= 1; i-- ) {
            int dof_id;
            LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
            int link_index = link_it->getLinkIndex();
         
            LinkMap::const_iterator parent_it = traversal.getParentLink(link_index);
            int parent_index = parent_it->getLinkIndex();
                
            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                dof_id = link_it->getAdjacentJoint(parent_it)->getDOFIndex();
                q_ = q(dof_id);
            } else {
                q_ = 0.0;
                dof_id = -1;
            } 
                    
            Ic[parent_index] = Ic[parent_index]+link_it->pose(parent_it,q_)*Ic[link_index];
            
            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                KDL::Twist S_link_parent = parent_it->S(link_it,q_);
                F = Ic[link_index]*S_link_parent;
                H(dof_id,dof_id) = dot(S_link_parent,F); 
                
                {
                    assert(parent_it != undirected_tree.getInvalidLinkIterator());
                    double q__;
                    int dof_id_;
                    LinkMap::const_iterator predecessor_it = traversal.getParentLink(link_it);
                    LinkMap::const_iterator successor_it = link_it;
                    while( true ) {
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            q__ = q( predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex());
                        } else {
                            q__ = 0.0;
                        } 
                        
                        F = successor_it->pose(predecessor_it,q__)*F;
                        
                        successor_it = predecessor_it;
                        predecessor_it = traversal.getParentLink(predecessor_it);
                        
                        if( predecessor_it == undirected_tree.getInvalidLinkIterator() ) {
                            break;
                        }
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            dof_id_ =  predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex();
                            q__ = q(dof_id_);
                        } else {
                            q__ = 0.0;
                            dof_id_ = -1;
                        } 
                        
                        Twist S_successor_predecessor = predecessor_it->S(successor_it,q__);
                        
                        if( dof_id_ >= 0 ) {
                            H(dof_id_,dof_id) = dot(S_successor_predecessor,F);
                            H(dof_id,dof_id_) = H(dof_id_,dof_id);
                        }
                        
                        
                    }
                }
                    
                    
                
            }
        }
        
        return 0;
    }
    
    
    int crba_floating_base_loop(const UndirectedTree & undirected_tree, const Traversal & traversal, const JntArray & q, std::vector<RigidBodyInertia> & Ic, FloatingJntSpaceInertiaMatrix & H) {
       double q_;
       Wrench F = Wrench::Zero();
        
        //Sweep from root to leaf
        for(int i=0;i<(int)traversal.getNrOfVisitedLinks();i++)
        {
          LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
          int link_index = link_it->getLinkIndex();
          
          //Collect RigidBodyInertia
          Ic[link_index]=link_it->getInertia();

        }
        
        for(int i=(int)traversal.getNrOfVisitedLinks()-1; i >= 1; i-- ) {
            int dof_id;
            LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
            int link_index = link_it->getLinkIndex();
         
            LinkMap::const_iterator parent_it = traversal.getParentLink(link_index);
            int parent_index = parent_it->getLinkIndex();
                
            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                dof_id = link_it->getAdjacentJoint(parent_it)->getDOFIndex();
                q_ = q(dof_id);
            } else {
                q_ = 0.0;
                dof_id = -1;
            } 
            
            RigidBodyInertia buf;
            buf = Ic[parent_index]+link_it->pose(parent_it,q_)*Ic[link_index];
            Ic[parent_index] = buf;
            

            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                KDL::Twist S_link_parent = parent_it->S(link_it,q_);
                F = Ic[link_index]*S_link_parent;
                H(6+dof_id,6+dof_id) = dot(S_link_parent,F); 
                
                if( traversal.getParentLink(link_it) != undirected_tree.getInvalidLinkIterator() ) {
                    double q__;
                    int dof_id_;
                    LinkMap::const_iterator predecessor_it = traversal.getParentLink(link_it);
                        LinkMap::const_iterator successor_it = link_it;
                    while(true) {
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            q__ = q( predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex());
                        } else {
                            q__ = 0.0;
                        } 
                        
                        #ifndef NDEBUG
                        //std::cout << "F migrated from frame " << successor_it->getLinkIndex() << " to frame " << successor_it->getLinkIndex() << std::endl;
                        #endif
                        F = successor_it->pose(predecessor_it,q__)*F;
                        
                        successor_it = predecessor_it;
                        predecessor_it = traversal.getParentLink(predecessor_it);
                        
                        if( predecessor_it == undirected_tree.getInvalidLinkIterator() ) { break; }
                        
                       
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            dof_id_ =  predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex();
                            q__ = q(dof_id_);
                        } else {
                            q__ = 0.0;
                            dof_id_ = -1;
                        } 
                        
                        Twist S_successor_predecessor = predecessor_it->S(successor_it,q__);
                        
                        if( dof_id_ >= 0 ) {
                            H(6+dof_id_,6+dof_id) = dot(S_successor_predecessor,F);
                            H(6+dof_id,6+dof_id_) = H(6+dof_id_,6+dof_id);
                        }
                        
                        
                    }
                    if( dof_id >= 0 ) { 
                        H.data.block(0,6+dof_id,6,1) = toEigen(F);
                        H.data.block(6+dof_id,0,1,6) = toEigen(F).transpose();
                    }
                     
                       
                    
                }
                    
            }
        }
        
        //The first 6x6 submatrix of the FlotingBase Inertia Matrix are simply the spatial inertia 
        //of all the structure expressed in the base reference frame
        H.data.block(0,0,6,6) = toEigen(Ic[traversal.getBaseLink()->getLinkIndex()]);
     
        return 0;
    }
    
   int crba_momentum_jacobian_loop(const UndirectedTree & undirected_tree, 
                                    const Traversal & traversal, 
                                    const JntArray & q, 
                                    std::vector<RigidBodyInertia> & Ic, 
                                    MomentumJacobian & H,
                                    RigidBodyInertia & InertiaCOM
                                   )
    {
        #ifndef NDEBUG
        if( undirected_tree.getNrOfLinks() != traversal.getNrOfVisitedLinks() ||
            undirected_tree.getNrOfDOFs() != q.rows() || 
            Ic.size() != undirected_tree.getNrOfLinks() ||
            H.columns() != (undirected_tree.getNrOfDOFs() + 6) ) 
        { 
            std::cerr << "crba_momentum_jacobian_loop: input data error" << std::endl;
            return -1; 
        } 
        #endif 
        
        double q_;
        Wrench F = Wrench::Zero();
        
        //Sweep from root to leaf
        for(int i=0;i<(int)traversal.getNrOfVisitedLinks();i++)
        {
          LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
          int link_index = link_it->getLinkIndex();
          
          //Collect RigidBodyInertia
          Ic[link_index]=link_it->getInertia();

        }
        
        for(int i=(int)traversal.getNrOfVisitedLinks()-1; i >= 1; i-- ) {
            int dof_id;
            LinkMap::const_iterator link_it = traversal.getOrderedLink(i);
            int link_index = link_it->getLinkIndex();
         
            LinkMap::const_iterator parent_it = traversal.getParentLink(link_index);
            int parent_index = parent_it->getLinkIndex();
                
            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                dof_id = link_it->getAdjacentJoint(parent_it)->getDOFIndex();
                q_ = q(dof_id);
            } else {
                q_ = 0.0;
                dof_id = -1;
            } 
            
            Ic[parent_index] = Ic[parent_index]+link_it->pose(parent_it,q_)*Ic[link_index];

            if( link_it->getAdjacentJoint(parent_it)->getNrOfDOFs() == 1 ) {
                KDL::Twist S_link_parent = parent_it->S(link_it,q_);
                F = Ic[link_index]*S_link_parent;
                
                if( traversal.getParentLink(link_it) != undirected_tree.getInvalidLinkIterator() ) {
                    double q__;
                    int dof_id_;
                    LinkMap::const_iterator predecessor_it = traversal.getParentLink(link_it);
                    LinkMap::const_iterator successor_it = link_it;
                    
                    while(true) {
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            q__ = q( predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex());
                        } else {
                            q__ = 0.0;
                        } 
                        
                        F = successor_it->pose(predecessor_it,q__)*F;
                        
                        successor_it = predecessor_it;
                        predecessor_it = traversal.getParentLink(predecessor_it);
                        
                        if( predecessor_it == undirected_tree.getInvalidLinkIterator() ) { break; }
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            dof_id_ =  predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex();
                            q__ = q(dof_id_);
                        } else {
                            q__ = 0.0;
                            dof_id_ = -1;
                        } 
                                                
                        
                    }
                    if( dof_id >= 0 ) { 
                        H.data.block(0,6+dof_id,6,1) = toEigen(F);
                    }
                     
                    //The first 6x6 submatrix of the Momentum Jacobian are simply the spatial inertia 
                    //of all the structure expressed in the base reference frame
                    H.data.block(0,0,6,6) = toEigen(Ic[traversal.getBaseLink()->getLinkIndex()]);
     
                    
                }
                    
            }
        }
        
        //We have then to translate the reference point of the obtained jacobian to the com
        //The Ic[traversal.order[0]->getLink(index)] contain the spatial inertial of all the tree
        //expressed in link coordite frames
        Vector com = Ic[traversal.getBaseLink()->getLinkIndex()].getCOG();
        H.changeRefPoint(com);
        
        InertiaCOM = Frame(com)*Ic[traversal.getBaseLink()->getLinkIndex()];
        
        return 0;
    }
}
}
