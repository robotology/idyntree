// 	Copyright	 (C)  2012  Wouter Bancken <wouter dot bancken at gmail dot com>
//   			 (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

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

#include <kdl/tree.hpp>
#include <sstream>
#include <algorithm>
#include <map>

#include "kdl_codyco/treegraph.hpp"

namespace KDL {
namespace CoDyCo {
    unsigned int TreeGraphLink::getNrOfAdjacentLinks() const
    {
        return adjacent_joint.size();
    }
     
    Frame TreeGraphLink::pose(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->second.pose(q,is_this_parent[adjacent_index]);
    }
    
    
    Twist TreeGraphLink::S(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->second.S(q,is_this_parent[adjacent_index]);
    }
    
    Twist TreeGraphLink::vj(int adjacent_index, const double& q,const double& qdot) const
    {
        return adjacent_joint[adjacent_index]->second.vj(q,qdot,is_this_parent[adjacent_index]);
    }
    
    void TreeGraphJoint::update_buffers(const double & q) const
    {
        if (q != q_previous) {
            relative_pose_parent_child = joint.pose(q)*f_tip;
            //Complicated and inefficient expression, but waiting for testing before simplifyng it (it is copied from working code, so it should work)
            S_child_parent = relative_pose_parent_child.M.Inverse(joint.twist(1.0).RefPoint(joint.pose(q).M * f_tip.p));
            S_parent_child = -(relative_pose_parent_child*S_child_parent);
            q_previous = q;
        }
    }
    
    Frame TreeGraphJoint::pose(const double& q, const bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return relative_pose_parent_child;
        } else {
            return relative_pose_parent_child.Inverse();
        }
    }
    
    Twist TreeGraphJoint::S(const double& q, bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return S_parent_child;
        } else {
            return S_child_parent;
        }
    }
    
    Twist TreeGraphJoint::vj(const double& q, const double &dq, bool inverse) const
    {
        return S(q,inverse)*dq;
    }
    
    LinkMap::iterator TreeGraph::getLink(const std::string& name, bool dummy)
    {
        return links.find(name);
    }
    
    JointMap::iterator TreeGraph::getJoint(const std::string& name, bool dummy)
    {
        return joints.find(name);
    }
    
    
    LinkMap::const_iterator TreeGraph::getLink(const std::string& name)
    {
        return links.find(name);
    }
    
    JointMap::const_iterator TreeGraph::getJoint(const std::string& name)
    {
        return joints.find(name);
    }
    

    
    TreeGraph::TreeGraph(const Tree & tree, TreeSerialization serialization)
    {
        if(!serialization.is_consistent(tree)) {
            serialization = TreeSerialization(tree);
        }
        
        nrOfDOFs = tree.getNrOfJoints();
        nrOfLinks = tree.getNrOfSegments();
        
        SegmentMap::const_iterator virtual_root, i, real_root;
        
        virtual_root = tree.getRootSegment();
        
        //If the virtual base is not fixed with the actual base
        //(or the virtual base has many children, so there is no actual base
        if( virtual_root->second.children.size() != 1 || virtual_root->second.children[0]->second.segment.getJoint().getType() == Joint::None ) {
            nrOfDOFs = 0;
            nrOfLinks = 0;
            return;
        }
        
        real_root = virtual_root->second.children[0];
        
        const SegmentMap& sm = tree.getSegments();
        
        //For loop to add link and joints
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            //Add link
            if( i != virtual_root ) {
                std::pair<std::string,TreeGraphLink> paar =
                    make_pair(current_segment.getName(), 
                        TreeGraphLink(current_segment.getName(),
                                      current_segment.getInertia(),
                                      serialization.getLinkId(current_segment.getName())));
                links.insert(paar);
            }
            
            //Add joint
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to TreeGraph
                const Joint & current_joint = current_segment.getJoint();
                std::pair< std::string , TreeGraphJoint > paar =
                    make_pair(current_joint.getName(), 
                              TreeGraphJoint(current_joint.getName(),
                                             current_joint, 
                                             current_joint.pose(0).Inverse()*current_segment.getFrameToTip(), /* Complicated way to do a simple thing: get f_tip attribute of the Segment */
                                             serialization.getJointId(current_joint.getName())));
                joints.insert(paar);
            }
        }
        
        //For loop to fix references between link and joints
         for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to TreeGraph
                const Joint & current_joint = current_segment.getJoint();
                JointMap::iterator tree_graph_joint = getJoint(current_joint.getName(),true);
                tree_graph_joint->second.parent = getLink(i->second.parent->first);
                tree_graph_joint->second.child = getLink(i->first);
                
                getLink(i->second.parent->first,true)->second.adjacent_joint.push_back(tree_graph_joint);
                getLink(i->second.parent->first,true)->second.is_this_parent.push_back(true);
                
                getLink(i->first,true)->second.adjacent_joint.push_back(tree_graph_joint);
                getLink(i->first,true)->second.is_this_parent.push_back(false);
            }
        }
        
        assert(nrOfLinks == links.size());
        
    } 
    
}
}//end of namespace
