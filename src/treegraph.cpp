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
#include <stack>
#include <iostream>
#include <cassert>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include "kdl_codyco/treegraph.hpp"

namespace KDL {
namespace CoDyCo {
    int TreeGraphLink::globalIterator2localIndex(LinkMap::const_iterator link_iterator) const
    {
        int i;
        for(i=0; i < (int)getNrOfAdjacentLinks(); i++ ) {
            if( adjacent_link[i] == link_iterator ) {
                break;
            }
        }
            assert(i >= 0);
            assert(i < (int)getNrOfAdjacentLinks());
            return i;
    }

    unsigned int TreeGraphLink::getNrOfAdjacentLinks() const
    {
        assert(adjacent_joint.size() == adjacent_link.size());
        return adjacent_joint.size();
    }
    
    bool TreeGraphLink::is_adjacent_to(LinkMap::const_iterator link_iterator) const
    {
        for(int i=0; i < (int)getNrOfAdjacentLinks(); i++ ) {
            if( adjacent_link[i] == link_iterator ) {
                return true;
            }
        }
        return false;
    }
     
    Frame & TreeGraphLink::pose(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->pose(q,is_this_parent[adjacent_index]);
    }
    
    Twist & TreeGraphLink::S(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->S(q,is_this_parent[adjacent_index]);
    }
    
    Twist TreeGraphLink::vj(int adjacent_index, const double& q,const double& qdot) const
    {
        return adjacent_joint[adjacent_index]->vj(q,qdot,is_this_parent[adjacent_index]);
    }
    
    Frame & TreeGraphLink::pose(LinkMap::const_iterator adjacent_iterator, const double& q) const
    {
        return pose(globalIterator2localIndex(adjacent_iterator),q);
    }
    
    
    Twist & TreeGraphLink::S(LinkMap::const_iterator adjacent_iterator, const double& q) const
    {
        return S(globalIterator2localIndex(adjacent_iterator),q);
    }
    
    Twist TreeGraphLink::vj(LinkMap::const_iterator adjacent_iterator, const double& q,const double& qdot) const
    {
        int adjacent_index = this->globalIterator2localIndex(adjacent_iterator);
        return vj(adjacent_index,q,qdot);
    }
    
    JunctionMap::const_iterator TreeGraphLink::getAdjacentJoint(int adjacent_index) const
    {
        return adjacent_joint[adjacent_index];
    }
    
    JunctionMap::const_iterator TreeGraphLink::getAdjacentJoint(LinkMap::const_iterator adjacent_iterator) const
    {
        int adjacent_index = globalIterator2localIndex(adjacent_iterator);
        return adjacent_joint[adjacent_index];
    }

    std::string TreeGraphLink::toString() const
    {
        std::stringstream ss;
        ss << link_name << " " << link_nr << " "  << " mass " << I.getMass();
        return ss.str();
    }
    
    void TreeGraphJunction::update_buffers(const double & q) const
    {
        if (q != q_previous) {
            relative_pose_parent_child = joint.pose(q)*f_tip;
            relative_pose_child_parent = relative_pose_parent_child.Inverse();
            //Complicated and inefficient expression, but waiting for testing before simplifyng it (it is copied from working code, so it should work)
            S_child_parent = relative_pose_parent_child.M.Inverse(joint.twist(1.0).RefPoint(joint.pose(q).M * f_tip.p));
            S_parent_child = -(relative_pose_parent_child*S_child_parent);
            q_previous = q;
        }
    }
    
    Frame & TreeGraphJunction::pose(const double& q, const bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return relative_pose_parent_child;
        } else {
            return relative_pose_child_parent;
        }
    }
    
    Twist & TreeGraphJunction::S(const double& q, bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return S_parent_child;
        } else {
            return S_child_parent;
        }
    }
    
    Twist TreeGraphJunction::vj(const double& q, const double &dq, bool inverse) const
    {
        return S(q,inverse)*dq;
    }
    
    std::string TreeGraphJunction::toString() const
    {
        std::stringstream ss;
        ss << joint_name << " " << q_nr << " "  << joint.getTypeName();
        return ss.str();
    }
    
    LinkMap::iterator TreeGraph::getLink(const std::string& name, bool dummy)
    {
        LinkNameMap::iterator ret_value = links_names.find(name);
        assert(ret_value != links_names.end());
        return ret_value->second;
    }
    
    JunctionMap::iterator TreeGraph::getJunction(const std::string& name, bool dummy)
    {       
        JunctionNameMap::iterator ret_value = junctions_names.find(name);
        #ifndef NDEBUG
        std::cerr << "Called getJunction with argument: " << name << std::endl;
        #endif 
        assert(junctions_names.size() == getNrOfJunctions());
        assert(ret_value != junctions_names.end());
        return ret_value->second;
    }
    
    LinkMap::const_iterator TreeGraph::getLink(const std::string& name) const
    {
        LinkNameMap::const_iterator ret_value = links_names.find(name);
        assert(ret_value != links_names.end());
        return ret_value->second;
    }
    
    LinkMap::const_iterator TreeGraph::getLink(const int index) const
    {
        assert(index >= 0 && index < getNrOfLinks());
        return links.begin()+index;
    }
        
    JunctionMap::const_iterator TreeGraph::getJunction(const int index) const
    {
        assert(index >= 0 && index < getNrOfJunctions());
        return junctions.begin()+index;
    }
    
    JunctionMap::const_iterator TreeGraph::getJunction(const std::string& name) const
    {
        JunctionNameMap::const_iterator ret_value = junctions_names.find(name);
        assert(ret_value != junctions_names.end());
        return ret_value->second;
    }

    
    TreeGraph::TreeGraph(const Tree & tree, const TreeSerialization & serialization, const TreePartition & partition)
    {
        TreeSerialization local_serialization = serialization;
        TreePartition local_partition = partition;
        
        #ifndef NDEBUG
        assert( local_serialization.is_consistent(tree) == serialization.is_consistent(tree) ); 
        if( local_serialization.is_consistent(tree)  ) {
            std::cout << "TreeGraph constructor: using provided serialization " << std::endl;
        } else {
            std::cout << "TreeGraph constructor: using default serialization " << std::endl;
        }
        #endif
        if( !local_serialization.is_consistent(tree) ) {
            local_serialization = TreeSerialization(tree);
        }
        
        if( !local_partition.is_consistent(tree,local_serialization) ) {
            local_partition = TreePartition(tree);
        }
        
        nrOfDOFs = tree.getNrOfJoints();
        nrOfLinks = tree.getNrOfSegments();
        
        links.resize(nrOfLinks);
        junctions.resize(nrOfLinks-1);
        
        SegmentMap::const_iterator virtual_root, i, real_root;
        
        virtual_root = tree.getRootSegment();
        
        //If the virtual base is not fixed with the actual base
        //(or the virtual base has many children, so there is no actual base
        if( virtual_root->second.children.size() != 1 || virtual_root->second.children[0]->second.segment.getJoint().getType() != Joint::None ) {
            #ifndef NDEBUG
            std::cerr << "TreeGraph constructor failed" << std::endl;
            #endif
            nrOfDOFs = 0;
            nrOfLinks = 0;
            return;
        }
        
        real_root = virtual_root->second.children[0];
        
        #ifndef NDEBUG
        std::cerr << "TreeGraph:" << std::endl;
        std::cerr << "virtual_root " << virtual_root->first << std::endl;
        std::cerr << "real_root " << real_root->first << std::endl;
        std::cerr << "Serialization " << local_serialization.toString() << std::endl;
        #endif
        
        
        original_root = real_root->first;
        
        const SegmentMap& sm = tree.getSegments();
        
        //For loop to add link and joints
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            //Add link
            if( i != virtual_root ) {
                int link_id = local_serialization.getLinkId(current_segment.getName());
                int part_id = local_partition.getPartIDfromLink(link_id);
                        #ifndef NDEBUG
                        //std::cerr << "Added link " << paar.second.link_name <<  " to TreeGraph with link_nr " << paar.second.link_nr << 
                        //         "and mass " << paar.second.I.getMass() << " and cog " << paar.second.I.getCOG()(0) << std::endl;
                        #endif
                assert(link_id >= 0 && link_id < (int)getNrOfLinks());
                links[link_id] = 
                        TreeGraphLink(current_segment.getName(),
                                      current_segment.getInertia(),
                                      link_id,
                                      part_id);
                                      
                links_names.insert(make_pair(current_segment.getName(),links.begin()+link_id));
            }
            
            //Add joint
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to TreeGraph
                const Joint & current_joint = current_segment.getJoint();
                
                if( current_joint.getType() == Joint::None ) {
                    int junction_id = local_serialization.getJunctionId(current_joint.getName());
                    junctions[junction_id] =  TreeGraphJunction(current_joint.getName(),
                                                current_joint, 
                                                current_joint.pose(0).Inverse()*current_segment.getFrameToTip(),junction_id);
                                                
                    junctions_names.insert(make_pair(current_joint.getName(),junctions.begin()+junction_id));
                } else {
                    // \note !!! for now, for junction with DOF junction_id == dof_id
                    int dof_id = local_serialization.getDOFId(current_joint.getName());
                    int dof_part_ID = local_partition.getPartIDfromDOF(dof_id);
            
                    junctions[dof_id] = TreeGraphJunction(current_joint.getName(),
                                                current_joint, 
                                                current_joint.pose(0).Inverse()*current_segment.getFrameToTip(), /* Complicated way to do a simple thing: get f_tip attribute of the Segment */
                                                dof_id,
                                                dof_part_ID);
                                                
                    junctions_names.insert(make_pair(current_joint.getName(),junctions.begin()+dof_id));

                }
            }
        }
        
        //For loop to fix references between link and joints
         for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to TreeGraph
                const Joint & current_joint = current_segment.getJoint();
                JunctionMap::iterator tree_graph_junction = getJunction(current_joint.getName(),true);
                tree_graph_junction->parent = getLink(i->second.parent->second.segment.getName());
                tree_graph_junction->child = getLink(i->second.segment.getName());
                
                getLink(i->second.parent->first,true)->adjacent_joint.push_back(getJunction(current_joint.getName()));
                getLink(i->second.parent->first,true)->is_this_parent.push_back(true);
                assert(getLink(i->first,true)->link_nr >= 0);
                getLink(i->second.parent->first,true)->adjacent_link.push_back(getLink(i->first));
                #ifndef NDEBUG
                    //std::cerr << "\tAdded link " << getLink(i->first,true)->second.link_name <<  " link_nr " << getLink(i->first,true)->second.link_nr << 
                    //             "as neighbour of " <<getLink(i->second.parent->first,true)->second.link_name << " link_nr " << getLink(i->second.parent->first,true)->second.link_nr << std::endl;
                #endif

                getLink(i->first,true)->adjacent_joint.push_back(getJunction(current_joint.getName()));
                getLink(i->first,true)->is_this_parent.push_back(false);
                assert(getLink(i->second.parent->first,true)->link_nr >= 0);
                getLink(i->first,true)->adjacent_link.push_back(getLink(i->second.parent->first));
                    #ifndef NDEBUG
                    //std::cerr << "\tAdded link " << getLink(i->second.parent->first,true)->second.link_name <<  " link_nr " <<  getLink(i->second.parent->first,true)->second.link_nr << 
                    //             "as neighbour of " <<getLink(i->first,true)->second.link_name << " link_nr " << getLink(i->first,true)->second.link_nr  << std::endl;
                #endif
                //Avoid to have uninitialized buffer
                //KDL::Frame X = tree_graph_joint->second.pose(0.0,true);
                //KDL::Vector v(1,2,3);
                //do some operation to ensure this function call is not eliminated by optimization
                //v = X*v;
            }
        }
        
        #ifndef NDEBUG
        std::cerr << "Check consistency exiting TreeGraph constructor " << std::endl;
        //assert(check_consistency() == 0);
        #endif
        assert(nrOfLinks == (int)links.size());
        
    } 
    
    int TreeGraph::compute_traversal(Traversal & traversal, const std::string& base_link,const bool bf_traversal) const
    {
        if( traversal.order.size() != getNrOfLinks() ) traversal.order.reserve(getNrOfLinks());

        if( traversal.parent.size() != getNrOfLinks() ) traversal.parent.resize(getNrOfLinks());
        
        #ifndef NDEBUG
        std::cerr << "Check consistency at the begin of compute_traversal " << std::endl;
        //assert(check_consistency() == 0);
        #endif
        #ifndef NDEBUG
        std::cerr << "Original base " << original_root << std::endl;
        #endif
        
        LinkMap::const_iterator base;
        if( base_link.size() == 0 ) {
            base = getLink(original_root);
        } else {
            base = getLink(base_link);
        }
        if( base == links.end() ) {
            return -1;
        }
        
        std::deque<LinkMap::const_iterator> to_visit;
        to_visit.clear();
        traversal.order.clear();
        
        to_visit.push_back(base);
        traversal.parent[base->link_nr] = getInvalidLinkIterator();
        
        LinkMap::const_iterator visited_link;
        LinkMap::const_iterator visited_child;
        
        
            #ifndef NDEBUG
            //std::cerr << "traversal.parent.size() " << traversal.parent.size() << std::endl;
            #endif
        
        while( to_visit.size() > 0 ) {
            #ifndef NDEBUG
            //std::cerr << "to_visit size: " << to_visit.size() << std::endl;
            #endif
            if( !bf_traversal ) {
                //Depth first : to_visit is a stack
                visited_link = to_visit.back();
                to_visit.pop_back();
            } else {
                //Breath first : to_visit id a queue
                visited_link = to_visit.front();
                to_visit.pop_front();
            }
            
            traversal.order.push_back(visited_link);
            
            #ifndef NDEBUG
            //std::cerr << "Going to add child of visited_link->second.link_nr " << visited_link->second.link_nr
            //          << " " << visited_link->second.getNrOfAdjacentLinks() 
            //          << " "  << visited_link->second.link_name << std::endl;
            #endif
            for(int i=0; i < (int)visited_link->getNrOfAdjacentLinks(); i++) {
                visited_child = visited_link->adjacent_link[i];
                assert(visited_child != links.end());
                assert(visited_child->link_nr >= 0);
                if( visited_child != traversal.parent[visited_link->link_nr] ) {
                    to_visit.push_back(visited_child);
                    #ifndef NDEBUG
                    //std::cerr << "Goint to add to_visit link " << visited_child->second.link_nr << std::endl; 
                    #endif 
                    traversal.parent[visited_child->link_nr] = visited_link;
                    #ifndef NDEBUG
                    //std::cerr << "Add to_visit link " << visited_child->second.link_nr << std::endl; 
                    #endif 
                }
            }
        }
        
        #ifndef NDEBUG
        //std::cerr << "Traversal order: " << std::endl;
        //for(int i=0; i < traversal.order.size(); i++ ) {
        //    std::cerr << traversal.order[i]->second.link_name << " " << traversal.order[i]->second.link_nr << std::endl;
        //}
        #endif
        
        
        
        return 0;
    }

    
    TreeSerialization TreeGraph::getSerialization() const
    {
        TreeSerialization ret;
        ret.links.resize(getNrOfLinks());
        ret.dofs.resize(getNrOfDOFs());
        ret.junctions.resize(getNrOfJunctions());
        
        for(LinkMap::const_iterator it=links.begin(); it != links.end(); it++ ) {
            ret.links[it->link_nr] = it->getName();
        }
        
        for(JunctionMap::const_iterator it=junctions.begin(); it != junctions.end(); it++ ) {
            if( it->joint.getType() != Joint::None ) {
                ret.dofs[it->q_nr] = it->getName();
            }
            ret.junctions[it->q_nr] = it->getName();
        }
        
        return ret;
    }
    
    int TreeGraph::check_consistency() const
    {
        LinkMap::const_iterator link_it;
        JunctionMap::const_iterator junction_it;
        
        for(link_it = links.begin(); link_it != links.end(); link_it++) {
            assert(link_it->link_nr >= 0 && link_it->link_nr < getNrOfLinks());
            #ifndef NDEBUG
            std::cerr << "Considering link " << link_it->link_name << " " << link_it->link_nr << std::endl;
            #endif
            for(int i=0; i < (int)link_it->getNrOfAdjacentLinks(); i++ ) {
                #ifndef NDEBUG
                std::cerr << "\tHas joint connecting parent " << link_it->adjacent_joint[i]->parent->link_name << "  and  child" << link_it->adjacent_joint[i]->child->link_name << std::endl;
                //std::cerr << link_it->adjacent_joint[i]->second.joint.pose(1.0);
                //std::cerr << link_it->adjacent_joint[i]->second.pose(1.0,true);
                std::cerr << link_it->pose(i,7.0);
                if( link_it->adjacent_joint[i]->child != link_it ) {
                    //std::cout << link_it->second.link_name<< " is not " << link_it->second.adjacent_joint[i]->second.child->second.link_name  << std::endl;
                    //std::cout << link_it->first<< " is not " << link_it->second.adjacent_joint[i]->second.child->first  << std::endl;
                }
                if( link_it != getLink("eyes_tilt_link") ) {
                    //std::cout << link_it->first << " is not " << getLink("eyes_tilt_link")->first << std::endl;
                }
                //std::cerr << "\tConsidering neighbour " << link_it->second.adjacent_link[i]->second.link_name << " " << link_it->second.adjacent_link[i]->second.link_nr << std::endl;
                #endif 
                assert((link_it->adjacent_link[i] ==  link_it->adjacent_joint[i]->parent && link_it ==  link_it->adjacent_joint[i]->child) || (link_it ==  link_it->adjacent_joint[i]->parent &&  link_it->adjacent_link[i] ==  link_it->adjacent_joint[i]->child)) ;
            }
        }
        
                
        for(junction_it = junctions.begin(); junction_it != junctions.end(); junction_it++) {
            if(junction_it->joint.getType() != Joint::None ) {
                assert(junction_it->q_nr >= 0 && junction_it->q_nr < (int)getNrOfDOFs());
            }
        }
        
        return 0;
    }
    
    int TreeGraph::check_consistency(const Traversal traversal) const
    {
        assert( traversal.order.size() == getNrOfLinks() );

        assert( traversal.parent.size() == getNrOfLinks() );
        
        LinkMap::const_iterator link_it;
        
        for(link_it = links.begin(); link_it != links.end(); link_it++) {
            if( traversal.parent[link_it->link_nr] == getInvalidLinkIterator() ) {
                if( link_it != traversal.order[0] ) return -1;
            } else {
                if( !( link_it->is_adjacent_to(traversal.parent[link_it->link_nr]) ) ) return -1;
            }
        }
        
        return 0;
    }
    
    std::string TreeGraph::toString() const
    {
        std::stringstream ss;
        ss << "TreeGraph " << tree_name << " original_root " << original_root << " DOFs " <<  nrOfDOFs << " nrOfLinks " << nrOfLinks << std::endl;
        ss << "Links: " << std::endl;
        for(LinkMap::const_iterator link_it = links.begin(); link_it != links.end(); link_it++) {
            ss << link_it->toString() << std::endl;
        }
        ss << "Joints: " << std::endl;
        for(JunctionMap::const_iterator junction_it = junctions.begin(); junction_it != junctions.end(); junction_it++) {
            ss << junction_it->toString() << std::endl;
        }
        return ss.str();
    }
    
}
}//end of namespace
