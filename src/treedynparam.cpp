// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

// Version: 1.0
// Author: Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>
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

#include <kdl_codyco/treedynparam.hpp>
//#include "frames_io.hpp"
//#include <iostream>

#include <kdl_codyco/regressor_utils.hpp>
#include <Eigen/Dense>

#ifndef NDEBUG
#include <iostream>
#endif

namespace KDL {
namespace CoDyCo {
    
    TreeDynParam::TreeDynParam(const Tree& _tree, Vector _grav, const TreeSerialization & _serialization):
        UndirectedTreeSolver(_tree,_serialization),
        grav(_grav),
        treeidsolver_coriolis( _tree, Vector::Zero()),
        treeidsolver_gravity( _tree, grav),
        nj(_tree.getNrOfJoints()),
        ns(_tree.getNrOfSegments()),
        jntarraynull(nj),
        wrenchnull(ns,Wrench::Zero()),
        Ic(ns)
    {
        ag=-Twist(grav,Vector::Zero());
        
        #ifndef NDEBUG
        std::cout << "Allocate TreeDynParam, used undirected_tree: " << std::endl;
        std::cout << undirected_tree.toString() << std::endl;
        assert(undirected_tree.check_consistency() == 0);
        assert(undirected_tree.check_consistency(traversal) == 0);
        #endif
    }

    //calculate inertia matrix H
    int TreeDynParam::JntToMass(const JntArray &q, JntSpaceInertiaMatrix& H)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=6 || H.columns()!=6 )
            return -1;
        
        SetToZero(H);
        
        unsigned int k=0;
        double q_;
        Wrench F;
        
        //Sweep from root to leaf
        for(int i=0;i<(int)traversal.order.size();i++)
        {
          LinkMap::const_iterator link_it = traversal.order[i];
          int link_index = link_it->getLinkIndex();
          
          //Collect RigidBodyInertia
          Ic[link_index] = link_it->getInertia();
          /*
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              q_=q(k);
              k++;
          }
          else
          {
            q_=0.0;
          }
          X[i]=chain.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
          S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));  
          */
        }
        
        for(int i=(int)traversal.order.size()-1; i >= 1; i-- ) {
            int dof_id;
            LinkMap::const_iterator link_it = traversal.order[i];
            int link_index = link_it->getLinkIndex();
         
            LinkMap::const_iterator parent_it = traversal.parent[link_index];
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
                    LinkMap::const_iterator predecessor_it = traversal.parent[link_it->getLinkIndex()];
                    LinkMap::const_iterator successor_it = link_it;
                    while( true ) {
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            q__ = q( predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex());
                        } else {
                            q__ = 0.0;
                        } 
                        
                        F = successor_it->pose(predecessor_it,q__)*F;
                        
                        successor_it = predecessor_it;
                        predecessor_it = traversal.parent[predecessor_it->getLinkIndex()];
                        
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
        
        /*
        for(unsigned int i=0;i<ns;i++)
        {
          //Collect RigidBodyInertia
          Ic[i]=chain.getSegment(i).getInertia();
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              q_=q(k);
              k++;
          }
          else
          {
            q_=0.0;
          }
          X[i]=chain.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
          S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));  
        }
        //Sweep from leaf to root
        int j,l;
        k=nj-1; //reset k
        for(int i=ns-1;i>=0;i--)
        {
          
          if(i!=0)
            {
              //assumption that previous segment is parent
              Ic[i-1]=Ic[i-1]+X[i]*Ic[i];
            } 

          F=Ic[i]*S[i];
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              H(k,k)=dot(S[i],F);
              j=k; //countervariable for the joints
              l=i; //countervariable for the segments
              while(l!=0) //go from leaf to root starting at i
                {
                  //assumption that previous segment is parent
                  F=X[l]*F; //calculate the unit force (cfr S) for every segment: F[l-1]=X[l]*F[l]
                  l--; //go down a segment
                  
                  if(chain.getSegment(l).getJoint().getType()!=Joint::None) //if the joint connected to segment is not a fixed joint
                  {    
                    j--;
                    H(k,j)=dot(F,S[l]); //here you actually match a certain not fixed joint with a segment 
                    H(j,k)=H(k,j);
                  }
                } 
              k--; //this if-loop should be repeated nj times (k=nj-1 to k=0)
          }

        }
        */
        
        
        return 0;
    }

    //calculate coriolis torques C
    int TreeDynParam::JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);

        //the calculation of coriolis matrix C
        return treeidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis);
        
    }

    //calculate gravity torques G
    int TreeDynParam::JntToGravity(const JntArray &q,JntArray &gravity)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);
        //the calculation of coriolis matrix C
        return treeidsolver_gravity.CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity);
    }
    
    //calculate coriolis torques 
    int TreeDynParam::JntToCoriolis(const JntArray &q, const JntArray &q_dot, const Twist & base_vel,  JntArray &coriolis_torques, Wrench & coriolis_base_wrench)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);

        //the calculation of coriolis matrix C
        return treeidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, base_vel, Twist::Zero(), wrenchnull, coriolis_torques, coriolis_base_wrench);
        
    }
    
       int TreeDynParam::JntToMass(const JntArray &q, FloatingJntSpaceInertiaMatrix& H)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=nj+6 || H.columns()!=nj+6 )
            return -1;
        
        double q_;
        Wrench F = Wrench::Zero();
        
        //Sweep from root to leaf
        for(int i=0;i<(int)traversal.order.size();i++)
        {
          LinkMap::const_iterator link_it = traversal.order[i];
          int link_index = link_it->getLinkIndex();
          
          //Collect RigidBodyInertia
          Ic[link_index]=link_it->getInertia();
          /*
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              q_=q(k);
              k++;
          }
          else
          {
            q_=0.0;
          }
          X[i]=chain.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
          S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));  
          */
        }
        
        for(int i=(int)traversal.order.size()-1; i >= 1; i-- ) {
            int dof_id;
            LinkMap::const_iterator link_it = traversal.order[i];
            int link_index = link_it->getLinkIndex();
         
            LinkMap::const_iterator parent_it = traversal.parent[link_index];
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
                
                if( traversal.parent[link_it->getLinkIndex()] != undirected_tree.getInvalidLinkIterator() ) {
                    double q__;
                    int dof_id_;
                    LinkMap::const_iterator predecessor_it = traversal.parent[link_it->getLinkIndex()];
                        LinkMap::const_iterator successor_it = link_it;
                    while(true) {
                        
                        if( predecessor_it->getAdjacentJoint(successor_it)->getNrOfDOFs() == 1 ) {
                            q__ = q( predecessor_it->getAdjacentJoint(successor_it)->getDOFIndex());
                        } else {
                            q__ = 0.0;
                        } 
                        
                        #ifndef NDEBUG
                        std::cout << "F migrated from frame " << successor_it->getLinkIndex() << " to frame " << successor_it->getLinkIndex() << std::endl;
                        #endif
                        F = successor_it->pose(predecessor_it,q__)*F;
                        
                        successor_it = predecessor_it;
                        predecessor_it = traversal.parent[predecessor_it->getLinkIndex()];
                        
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
        H.data.block(0,0,6,6) = toEigen(Ic[traversal.order[0]->getLinkIndex()]);
        
        /*
        for(unsigned int i=0;i<ns;i++)
        {
          //Collect RigidBodyInertia
          Ic[i]=chain.getSegment(i).getInertia();
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              q_=q(k);
              k++;
          }
          else
          {
            q_=0.0;
          }
          X[i]=chain.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
          S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));  
        }
        //Sweep from leaf to root
        int j,l;
        k=nj-1; //reset k
        for(int i=ns-1;i>=0;i--)
        {
          
          if(i!=0)
            {
              //assumption that previous segment is parent
              Ic[i-1]=Ic[i-1]+X[i]*Ic[i];
            } 

          F=Ic[i]*S[i];
          if(chain.getSegment(i).getJoint().getType()!=Joint::None)
          {
              H(k,k)=dot(S[i],F);
              j=k; //countervariable for the joints
              l=i; //countervariable for the segments
              while(l!=0) //go from leaf to root starting at i
                {
                  //assumption that previous segment is parent
                  F=X[l]*F; //calculate the unit force (cfr S) for every segment: F[l-1]=X[l]*F[l]
                  l--; //go down a segment
                  
                  if(chain.getSegment(l).getJoint().getType()!=Joint::None) //if the joint connected to segment is not a fixed joint
                  {    
                    j--;
                    H(k,j)=dot(F,S[l]); //here you actually match a certain not fixed joint with a segment 
                    H(j,k)=H(k,j);
                  }
                } 
              k--; //this if-loop should be repeated nj times (k=nj-1 to k=0)
          }

        }
        */
        
        
        return 0;
    }
    
    TreeDynParam::~TreeDynParam()
    {
    }


}
}
