// Copyright  (C)  2013 Silvio Traversaro <silvio dot traversaro at iit dot it>
// Author: Silvio Traversaro

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
#include "kdl_codyco/undirectedtree.hpp"
#include <kdl_codyco/utils.hpp>

namespace KDL {
namespace CoDyCo {

    
    LinkMap::iterator UndirectedTree::getLink(const std::string& name, bool dummy)
    {
        LinkNameMap::iterator ret_value = links_names.find(name);

        assert(ret_value != links_names.end());
        return ret_value->second;
    }
    
    JunctionMap::iterator UndirectedTree::getJunction(const std::string& name, bool dummy)
    {       
        JunctionNameMap::iterator ret_value = junctions_names.find(name);
        #ifndef NDEBUG
        //std::cerr << "Called getJunction with argument: " << name << std::endl;
        //std::cerr << "junction_names.size() " << junctions_names.size() << " getNrOfJunctions " << getNrOfJunctions() << std::endl;
        #endif 
        assert(junctions_names.size() == getNrOfJunctions());
        assert(ret_value != junctions_names.end());
        return ret_value->second;
    }
    
    LinkMap::const_iterator UndirectedTree::getLink(const std::string& name) const
    {
        LinkNameMap::const_iterator ret_value = links_names.find(name);
        if( ret_value == links_names.end() ) {
            //std::cerr << "UndirectedTree: Link " << name << " not found " << std::endl; 
            return getInvalidLinkIterator();
        }
        return ret_value->second;
    }
    
    LinkMap::const_iterator UndirectedTree::getLink(const int index) const
    {
        if(!(index >= 0 && index < (int)getNrOfLinks())) {
            return getInvalidLinkIterator();
        }
        return links.begin()+index;
    }
        
    JunctionMap::const_iterator UndirectedTree::getJunction(const int index) const
    {
        if( !(index >= 0 && index < (int)getNrOfJunctions()) ) {
            return getInvalidJunctionIterator();
        }
        return junctions.begin()+index;
    }
    
    JunctionMap::const_iterator UndirectedTree::getJunction(const std::string& name) const
    {
        JunctionNameMap::const_iterator ret_value = junctions_names.find(name);
        if(ret_value == junctions_names.end()) {
            return getInvalidJunctionIterator();
        }
        return ret_value->second;
    }
    
    void UndirectedTree::constructor(const Tree & tree, const TreeSerialization & serialization, const TreePartition & partition)
    {
        TreeSerialization local_serialization = serialization;
        TreePartition local_partition = partition;
        
        #ifndef NDEBUG
        assert( local_serialization.is_consistent(tree) == serialization.is_consistent(tree) ); 
        if( local_serialization.is_consistent(tree)  ) {
            //std::cout << "UndirectedTree constructor: using provided serialization " << std::endl;
        } else {
            //std::cout << "UndirectedTree constructor: using default serialization " << std::endl;
        }
        #endif
        if( !local_serialization.is_consistent(tree) ) {
            local_serialization = TreeSerialization(tree);
        }
        
        if( !local_partition.is_consistent(tree,local_serialization) ) {
            local_partition = TreePartition(tree);
        }
        
  
        
        SegmentMap::const_iterator virtual_root, i, real_root;
        
        virtual_root = tree.getRootSegment();
        
        const SegmentMap& sm = tree.getSegments();
        
        int nrOfLinks;
        
        //If the virtual base is not fixed with the actual base
        //(or the virtual base has many children, so there is no actual base)
        //Insert a dummy base link explicitly in the constructed UndirectedTree
        if( virtual_root->second.children.size() != 1 || virtual_root->second.children[0]->second.segment.getJoint().getType() != Joint::None ) {
            #ifndef NDEBUG
            std::cerr << "UndirectedTree constructor: no fake base found" << std::endl;
            #endif
            real_root = virtual_root;
            virtual_root = sm.end();
            nrOfLinks = tree.getNrOfSegments()+1;
        } else {
            real_root = virtual_root->second.children[0];
            nrOfLinks = tree.getNrOfSegments();
        }
        
        nrOfDOFs = tree.getNrOfJoints();
        
        
        links.resize(nrOfLinks);
        junctions.resize(nrOfLinks-1);
        
        
        #ifndef NDEBUG
        //std::cerr << "UndirectedTree:" << std::endl;
        //std::cerr << "virtual_root " << virtual_root->first << std::endl;
        //std::cerr << "real_root " << real_root->first << std::endl;
        //std::cerr << "Serialization " << local_serialization.toString() << std::endl;
        #endif
        
        
        //Add part names
        for(int l=0; l < local_partition.getNrOfParts(); l++ ) {
            TreePart part = local_partition.getPartFromLocalIndex(l);
            body_part_names.insert(make_pair(part.getPartID(),part.getPartName()));
        }
        
        
        original_root = real_root->first;
        
        
   
        
        //For loop to add link and joints
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            //Add link
            if( i != virtual_root ) {
                int link_id = local_serialization.getLinkID(current_segment.getName());
                int part_id = local_partition.getPartIDfromLink(link_id);
                int local_link_id = local_partition.getLocalLinkIndex(link_id);
                
                assert( link_id >= 0 && link_id <= nrOfLinks );
                #ifndef NDEBUG
                if( part_id < 0 ) { std::cerr << "Error: " << current_segment.getName() << " has no part "  << std::endl; } 
                #endif
                assert( part_id >= 0 );
                assert( local_link_id >= 0 && local_link_id <= nrOfLinks );


    
                #ifndef NDEBUG
                //std::cerr << "Added link " << current_segment.getName() <<  " to UndirectedTree with link_nr " << link_id << 
                //         "and mass " << current_segment.getInertia().getMass() << " and cog " << current_segment.getInertia().getCOG()(0) << std::endl;
                #endif
                assert(link_id >= 0 && link_id < (int)getNrOfLinks());
                links[link_id] = 
                        UndirectedTreeLink(current_segment.getName(),
                                      current_segment.getInertia(),
                                      link_id,
                                      part_id,local_link_id);
                                      
                links_names.insert(make_pair(current_segment.getName(),links.begin()+link_id));
            }
            
            //Add joint
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to UndirectedTree
                const Joint & current_joint = current_segment.getJoint();
               
                
                if( current_joint.getType() == Joint::None ) {
                    int junction_id = local_serialization.getJunctionID(current_joint.getName());
                    //std::cout << "failed to find joint " << current_joint.getName() << std::endl;
                    //std::cout << "in serialization " << local_serialization.toString() << std::endl;
                    assert(junction_id >= 0);
                    junctions[junction_id] =  UndirectedTreeJunction(current_joint.getName(),
                                                current_joint, 
                                                current_joint.pose(0).Inverse()*current_segment.getFrameToTip(),junction_id);
                                                
                    junctions_names.insert(make_pair(current_joint.getName(),junctions.begin()+junction_id));
                } else {
                    // \note !!! for now, for junction with DOF junction_id == dof_id
                    int dof_id = local_serialization.getDOFID(current_joint.getName());
                    int dof_part_ID = local_partition.getPartIDfromDOF(dof_id);
                    int local_dof_id = local_partition.getLocalDOFIndex(dof_id);
                    //assert(dof_part_ID >= 0);
                    //assert(local_dof_id >= 0);
            
                    junctions[dof_id] = UndirectedTreeJunction(current_joint.getName(),
                                                current_joint, 
                                                current_joint.pose(0).Inverse()*current_segment.getFrameToTip(), /* Complicated way to do a simple thing: get f_tip attribute of the Segment */
                                                dof_id,
                                                dof_part_ID,local_dof_id);
                                                
                    junctions_names.insert(make_pair(current_joint.getName(),junctions.begin()+dof_id));

                }
            }
        }
        
        
        
        //For loop to fix references between link and joints
         for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            const Segment & current_segment =  i->second.segment;
            
            if( i != virtual_root && i != real_root ) {
                //If the father is the root, dont'add any joint
                //Add segment joint to UndirectedTree
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
        
        //std::cerr << "UndirectedTree::constructor() : check consistency exiting UndirectedTree constructor " << std::endl;
        //std::cerr << this->toString() << std::endl;
        assert(check_consistency() == 0);
        #endif
        assert(nrOfLinks == (int)links.size());
    }
    
    UndirectedTree::UndirectedTree(const Tree & tree, const TreeSerialization & serialization, const TreePartition & partition)
    {
        constructor(tree,serialization,partition);
    } 
    
    UndirectedTree::UndirectedTree(const UndirectedTree& in) 
    {
        constructor(in.getTree(),in.getSerialization(),in.getPartition());
    }

    UndirectedTree& UndirectedTree::operator=(const UndirectedTree& in) 
    {
        constructor(in.getTree(),in.getSerialization(),in.getPartition());
        return *this;
    }

    
    int UndirectedTree::compute_traversal(Traversal & traversal, const int base_link_index,const bool bf_traversal) const
    {
        if( traversal.order.capacity() < getNrOfLinks() ) traversal.order.reserve(getNrOfLinks());

        if( traversal.parent.size() != getNrOfLinks() ) traversal.parent.resize(getNrOfLinks());
        
        #ifndef NDEBUG
        //std::cerr << "Check consistency at the begin of compute_traversal " << std::endl;
        assert(check_consistency() == 0);
        #endif
        #ifndef NDEBUG
        //std::cerr << "Original base " << original_root << std::endl;
        #endif
        
        LinkMap::const_iterator base;
        if( base_link_index == COMPUTE_TRAVERSAL_BASE_LINK_DEFAULT_VALUE) {
            base = getLink(original_root);
        } else {
            base = getLink(base_link_index);
        }
        if( base == links.end() ) {
            std::cerr << "UndirectedTree::compute_traversal error: link with index " << base_link_index << " not found " << std::endl;
            return -1;
        }
        
        std::deque<LinkMap::const_iterator> to_visit;
        to_visit.clear();
        
        traversal.order.clear();
        
        to_visit.push_back(base);
        #ifndef NDEBUG
        //std::cerr << "Original base link_nr " << base->link_nr << std::endl;
        //std::cerr << "Traversal.parent size " << traversal.parent.size() << std::endl;
        #endif
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
            /*
            std::cerr << "Going to add child of visited_link->second.link_nr " << visited_link->getLinkIndex()
                      << " " << visited_link->getNrOfAdjacentLinks() 
                      << " "  << visited_link->getName() << std::endl;
            */
            #endif
            for(int i=0; i < (int)visited_link->getNrOfAdjacentLinks(); i++) {
                visited_child = visited_link->adjacent_link[i];
                assert(visited_child != links.end());
                assert(visited_child->link_nr >= 0);
                if( visited_child != traversal.parent[visited_link->link_nr] ) {
                    to_visit.push_back(visited_child);
                    #ifndef NDEBUG
                    //std::cerr << "Going to add to_visit link " << visited_child->getLinkIndex() << std::endl; 
                    #endif 
                    traversal.parent[visited_child->getLinkIndex()] = visited_link;
                    #ifndef NDEBUG
                    //std::cerr << "Add to_visit link " << visited_child->getLinkIndex() << std::endl; 
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
    
    int UndirectedTree::compute_traversal(Traversal & traversal, const std::string& base_link,const bool bf_traversal) const
    {
        #ifndef NDEBUG
        //std::cerr << "Called compute_traversal with " << base_link << "as base link " << std::endl;
        #endif
        LinkMap::const_iterator base_link_it = getLink(base_link);
        if( base_link_it == getInvalidLinkIterator() ) { 
            std::cerr << "UndirectedTree::compute_traversal error: link " << base_link << " not found " << std::endl;
            return -1;
        }
        return compute_traversal(traversal,base_link_it->getLinkIndex(),bf_traversal);
    }
    
    //Warning q_nr is dependent on the selected base, not on the serialization
    Tree UndirectedTree::getTree(std::string base) const
    {
        assert(this->check_consistency() == 0);
        
        //Define a KDL::Tree with fake link "base_link", as is usually done
        //in URDF describing humanoids
        const std::string fake_root_name = "base_link";

        KDL::Tree tree(fake_root_name);
        Traversal traversal;
        int ret;
        if( base.length() > 0 ) {
            ret  = compute_traversal(traversal,base);
        } else {
            ret = compute_traversal(traversal);
        }
        if( ret < 0 ) { std::cerr << "UndirectedTree::getTree : specified base " << base << " is not part of the UndirectedTree" << std::endl; return KDL::Tree("UndirectedTree_getTree_error");}
        
        assert(this->check_consistency() == 0);
        assert(this->check_consistency(traversal) == 0);
        
        for(int i=0; i < (int)traversal.order.size(); i++ ) {
            LinkMap::const_iterator link_it = traversal.order[i];
            if( i == 0 ) { 
                //The selected base should be attached rigidly to the fake "base_link"
                tree.addSegment(Segment(link_it->getName(),Joint("base_link_joint",Joint::None),KDL::Frame::Identity(),link_it->getInertia()),fake_root_name);
            } else {
                LinkMap::const_iterator parent_it = traversal.parent[link_it->link_nr];
                
                //The current link should be connected to his parent
                assert(link_it->is_adjacent_to(parent_it));
                JunctionMap::const_iterator junction_it = link_it->getAdjacentJoint(parent_it);
                
                Joint kdl_joint;
                KDL::Frame f_tip; 
                
                if( parent_it == junction_it->parent ) {
                    //If the parent was the parent in the original KDL::Tree
                    //we are adding the Joint in the "same" order of the original KDL::Tree
                    kdl_joint = junction_it->joint;
                    f_tip = junction_it->f_tip;
                    
                } else {
                    //otherwise, we have to invert the polarity of the joint
                    //std::cerr <<< "Calling JointInvertPolarity" << std::endl;
                    JointInvertPolarity(junction_it->joint,junction_it->f_tip,kdl_joint,f_tip);
                }
                
                //We are setting kdl_joint.pose(0)*f_tip so that the KDL::Segment f_tip is actually f_tip
                tree.addSegment(Segment(link_it->getName(),kdl_joint,kdl_joint.pose(0)*f_tip,link_it->getInertia()),parent_it->getName());
            }
        }
        
        return tree;
    }
    
    TreeSerialization UndirectedTree::getSerialization() const
    {
        TreeSerialization ret;
        ret.setNrOfLinks(getNrOfLinks());
        ret.setNrOfDOFs(getNrOfDOFs());
        ret.setNrOfJunctions(getNrOfJunctions());
        
        for(LinkMap::const_iterator it=links.begin(); it != links.end(); it++ ) {
            //ret.links[it->link_nr] = it->getName();
            ret.setLinkNameID(it->getName(),it->link_nr);
        }
        
        for(JunctionMap::const_iterator it=junctions.begin(); it != junctions.end(); it++ ) {
            if( it->joint.getType() != Joint::None ) {
                //ret.dofs[it->q_nr] = it->getName();
                ret.setDOFNameID(it->getName(),it->q_nr);
            }
            //ret.junctions[it->q_nr] = it->getName();
            ret.setJunctionNameID(it->getName(),it->q_nr);
        }
        
        return ret;
    }
    
    TreePartition UndirectedTree::getPartition() const
    {
        TreePartition partition;
        int i;
        
        std::vector<TreePart> parts(0);
        std::vector< std::vector<int> > part_dofs(0);
        std::vector< std::vector<int> > part_links(0);
         
        for(LinkMap::const_iterator it=links.begin(); it != links.end(); it++ ) {
            
            bool part_not_found = true;
            //Add the link to the right part
            for(i = 0; i < (int)parts.size(); i++ ) {
                if( parts[i].getPartID() == it->body_part_nr ) {
                    part_not_found = false;
                    break;
                }
            }
            //If the part was not already found, add the part
            if( part_not_found ) {
                std::map<int,std::string>::const_iterator bp_name_it = body_part_names.find(it->body_part_nr);
                if( bp_name_it == body_part_names.end() ) {
                    std::cerr << "UndirectedTree:getPartition() failed: body part " <<  it->body_part_nr << " not recognized (when adding link " << it->getName() << "  )\n" << std::endl;
                    assert(false);
                    return partition;
                }
                std::string part_name = bp_name_it->second;
                parts.push_back(TreePart(it->body_part_nr,part_name));
                part_dofs.push_back(std::vector<int>(0));
                part_links.push_back(std::vector<int>(0));
                part_not_found = false;
                assert(i == (int)parts.size()-1);
            }
            
            assert( i < (int)part_links.size() );
            assert( i < (int)part_dofs.size() );
            
            //Now that we have the part (it is parts[i]) we had the 
            //link to the link of the part.
            #ifndef NDEBUG
            //std::cerr << it->toString() << std::endl;
            #endif
            assert( it->body_part_link_nr >= 0 );
            if( it->body_part_link_nr >= (int)part_links[i].size() ) {
                part_links[i].resize(it->body_part_link_nr+1);
            }
            part_links[i][it->body_part_link_nr] = it->link_nr;
        }
        
        for(JunctionMap::const_iterator it=junctions.begin(); it != junctions.end(); it++ ) {
            if( it->joint.getType() != Joint::None ) {
                assert( it->body_part >= 0 );
                bool part_not_found = true;
                //Add the link to the right part
                for(i = 0; i < (int)parts.size(); i++ ) {
                    if( parts[i].getPartID() == it->body_part ) {
                        part_not_found = false;
                        break;
                    }
                }
                //If the part was not already found, add the part
                if( part_not_found ) {
                    std::map<int,std::string>::const_iterator bp_name_it = body_part_names.find(it->body_part);
                    if( bp_name_it == body_part_names.end() ) {
                        std::cerr << "UndirectedTree:getPartition() failed: body part " <<  it->body_part << " not recognized\n" << std::endl;
                        assert(false);        
                        return partition;
                    }
                    std::string part_name = bp_name_it->second;
                    parts.push_back(TreePart(it->body_part,part_name));
                    part_dofs.push_back(std::vector<int>(0));
                    part_links.push_back(std::vector<int>(0));
                    part_not_found = false;
                    assert(i == (int)parts.size()-1);
                }
                
                
                //Now that we have the part (it is parts[i]) we had the 
                //dof to the dofs of the part.
                if( it->body_part_q_nr >= (int)part_dofs[i].size() ) {
                    part_dofs[i].resize(it->body_part_q_nr+1);
                }
                part_dofs[i][it->body_part_q_nr] = it->q_nr;
            }
        }
        
        
        for(int j=0; j < (int)parts.size(); j++ ) {
            parts[j] = TreePart(parts[j].getPartID(),parts[j].getPartName(),part_dofs[j],part_links[j]);
            partition.addPart(parts[j]);
        }
        //add dofs to part
        
        return partition;
    }

    
    
    int UndirectedTree::check_consistency() const
    {
        LinkMap::const_iterator link_it;
        JunctionMap::const_iterator junction_it;
        
        for(link_it = links.begin(); link_it != links.end(); link_it++) {
            assert(link_it->link_nr >= 0 && link_it->link_nr < (int)getNrOfLinks());
            assert(link_it->body_part_link_nr >= 0 && link_it->body_part_link_nr < (int)getNrOfLinks());
            assert(link_it->body_part_nr >= 0);
            
            #ifndef NDEBUG
            //std::cerr << "Considering link " << link_it->link_name << " " << link_it->link_nr << std::endl;
            #endif
            for(int i=0; i < (int)link_it->getNrOfAdjacentLinks(); i++ ) {
                #ifndef NDEBUG
                //std::cerr << "\tHas joint connecting parent " << link_it->adjacent_joint[i]->parent->link_name << "  and  child" << link_it->adjacent_joint[i]->child->link_name << std::endl;
                //std::cerr << link_it->adjacent_joint[i]->second.joint.pose(1.0);
                //std::cerr << link_it->adjacent_joint[i]->second.pose(1.0,true);
                if( link_it->adjacent_joint[i]->child != link_it ) {
                    //std::cout << link_it->second.link_name<< " is not " << link_it->second.adjacent_joint[i]->second.child->second.link_name  << std::endl;
                    //std::cout << link_it->first<< " is not " << link_it->second.adjacent_joint[i]->second.child->first  << std::endl;
                }
       
                //std::cerr << "\tConsidering neighbour " << link_it->second.adjacent_link[i]->second.link_name << " " << link_it->second.adjacent_link[i]->second.link_nr << std::endl;
                #endif 
                assert((link_it->adjacent_link[i] ==  link_it->adjacent_joint[i]->parent && link_it ==  link_it->adjacent_joint[i]->child) || (link_it ==  link_it->adjacent_joint[i]->parent &&  link_it->adjacent_link[i] ==  link_it->adjacent_joint[i]->child)) ;
            }
        }
        
                
        for(junction_it = junctions.begin(); junction_it != junctions.end(); junction_it++) {
            if(junction_it->joint.getType() != Joint::None ) {
                if( !(junction_it->q_nr >= 0 && junction_it->q_nr < (int)getNrOfDOFs()) )  return -1;
                if( !(junction_it->body_part_q_nr >= 0 && junction_it->body_part_q_nr < (int)getNrOfDOFs()) ) return -1;
                if( !(junction_it->body_part >= 0) ) return -1;
            }
        }
        
        return 0;
    }
    
    int UndirectedTree::check_consistency(const Traversal traversal) const
    {
        if( traversal.order.size() != getNrOfLinks() ) return -1;
        if( traversal.order.size() != getNrOfLinks() ) return -1;

        assert( traversal.parent.size() == getNrOfLinks() );
        if( traversal.parent.size() != getNrOfLinks() ) return -1;

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
    
    std::string UndirectedTree::toString() const
    {
        std::stringstream ss;
        ss << "UndirectedTree " << tree_name << " original_root " << original_root << " DOFs " <<  nrOfDOFs << " nrOfLinks " << getNrOfLinks() << std::endl;
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
