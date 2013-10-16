/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
  
#include "kdl_codyco/treeserialization.hpp"
#include "kdl_codyco/tree_rotation.hpp"
#include <kdl/joint.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>


namespace KDL {
namespace CoDyCo {
    void TreeSerialization::addDFSrecursive(SegmentMap::const_iterator current_el, int & link_cnt, int & fixed_joints_cnt )
    {
        #ifndef NDEBUG
        //std::cout << "addDFSrecursive: called with segment " << current_el->second.segment.getName() << " link_cnt " << link_cnt << " fixed_joints_cnt " << fixed_joints_cnt << std::endl; 
        //std::cout << "nr of junctions " << getNrOfJunctions() << " nr of DOFs " << getNrOfDOFs() << std::endl;
        #endif
        if( current_el->second.segment.getJoint().getType() != Joint::None ) {
            dofs[current_el->second.q_nr] = current_el->second.segment.getJoint().getName();
            junctions[current_el->second.q_nr] = current_el->second.segment.getJoint().getName();
        } else {
            assert((int)junctions.size() == getNrOfJunctions());
            assert(getNrOfDOFs()+fixed_joints_cnt < (int)junctions.size());
            junctions[getNrOfDOFs()+fixed_joints_cnt] = current_el->second.segment.getJoint().getName();
            fixed_joints_cnt++;
        }
        
        links[link_cnt] = current_el->first;
        link_cnt++;
                
        for( unsigned int i=0; i < current_el->second.children.size(); i++ ) {
            addDFSrecursive(current_el->second.children[i],link_cnt,fixed_joints_cnt);
        }
        
    }
    
    void TreeSerialization::addDFSrecursive_only_fixed(SegmentMap::const_iterator current_el, int & fixed_joints_cnt )
    {
        if( current_el->second.segment.getJoint().getType() == Joint::None ) {
            junctions[getNrOfDOFs()+fixed_joints_cnt] = current_el->second.segment.getJoint().getName();
            fixed_joints_cnt++;
        }
        
        for( unsigned int i=0; i < current_el->second.children.size(); i++ ) {
            addDFSrecursive_only_fixed(current_el->second.children[i],fixed_joints_cnt);
        }
        
    }
    
    TreeSerialization::TreeSerialization() 
    {
        links = std::vector<std::string>(0);
        junctions = std::vector<std::string>(0);
        dofs = std::vector<std::string>(0);
    }

    TreeSerialization::~TreeSerialization() {}
    
    TreeSerialization::TreeSerialization(const Tree & tree) 
    {
        links = std::vector<std::string>(tree.getNrOfSegments());
        dofs = std::vector<std::string>(tree.getNrOfJoints());
        junctions = std::vector<std::string>(tree.getNrOfSegments()-1);
        
        
        SegmentMap::const_iterator root, real_root;
        SegmentMap::const_iterator child;
        
        int link_cnt = 0;
        
        int fixed_joints_cnt = 0;
        
         root = tree.getRootSegment();
        
        /** \todo remove this assumption */
        assert(root->second.children.size() != 0);
        SegmentMap::const_iterator root_child = root->second.children[0];
        
        //This should be coherent with the behaviour of UndirectedTree
        if( root->second.children.size() != 1 || root_child->second.segment.getJoint().getType() != Joint::None )
        {
            real_root = root;
            links.resize(links.size()+1);
            junctions.resize(junctions.size()+1);
        } else {
            real_root = root->second.children[0];
        }
        
        //Add real_root link without including fake joint
        links[link_cnt] = real_root->first;
        
        link_cnt++;
        
        for( unsigned int i=0; i < real_root->second.children.size(); i++ ) {
            addDFSrecursive(real_root->second.children[i],link_cnt,fixed_joints_cnt);
        }
        
        assert(this->is_consistent(tree));
    }
        
    TreeSerialization::TreeSerialization(const Tree & tree, std::vector<std::string> & links_in, std::vector<std::string> & joints_in) 
    {
        assert(links_in.size() == tree.getNrOfSegments());
        assert(joints_in.size() == tree.getNrOfSegments()-1 || joints_in.size() == tree.getNrOfJoints() );
        
        links = links_in;
        
        if( joints_in.size() == tree.getNrOfSegments()-1 ) {
            junctions = joints_in;
            
            dofs = junctions;
            
            dofs.resize(tree.getNrOfJoints());
        } else {
            dofs = joints_in;
            junctions = dofs;
            junctions.resize(tree.getNrOfSegments()-1);
            
            int fixed_joints_cnt = 0;
        
            SegmentMap::const_iterator root = tree.getRootSegment();
            for( unsigned int i=0; i < root->second.children.size(); i++ ) {
                addDFSrecursive_only_fixed(root->second.children[i],fixed_joints_cnt);
            }
        
        }
        
        assert(this->is_consistent(tree));
    }
    
    
    TreeSerialization::TreeSerialization(const TreeSerialization& x)
    {        
        links = std::vector<std::string>(0);
        junctions = std::vector<std::string>(0);
        dofs = std::vector<std::string>(0);
        links = x.links;
        junctions = x.junctions;
        dofs = x.dofs;
    }
    
    int TreeSerialization::getJunctionId(std::string junction_name) const
    {
        std::vector<std::string>::const_iterator it;
        it = std::find(junctions.begin(),junctions.end(),junction_name);
        if( it != junctions.end() ) {
            return it - junctions.begin();
        } else {
            return -1;
        }
    }
        
    int TreeSerialization::getLinkId(std::string link_name) const
    {
        std::vector<std::string>::const_iterator it;
        it = std::find(links.begin(),links.end(),link_name);
        if( it != links.end() ) {
            return it - links.begin();
        } else {
            return -1;
        }
    }
    
    int TreeSerialization::getDOFId(std::string dof_name) const
    {
        std::vector<std::string>::const_iterator it;
        it = std::find(dofs.begin(),dofs.end(),dof_name);
        if( it != dofs.end() ) {
            return it - dofs.begin();
        } else {
            return -1;
        }
    }
    
    std::string TreeSerialization::getDOFName(int dof_id) const
    {
        return dofs[dof_id];
    }
    
    std::string TreeSerialization::getJunctionName(int junction_id) const
    {
        return junctions[junction_id];
    }
    std::string TreeSerialization::getLinkName(int link_id) const
    {
        return links[link_id];
    }
    
    bool TreeSerialization::is_consistent(const Tree & tree) const
    {
        SegmentMap::const_iterator seg;
        
       SegmentMap::const_iterator root = tree.getRootSegment();
        
       /** \todo remove this assumption */
       assert(root->second.children.size() != 0);
       SegmentMap::const_iterator root_child = root->second.children[0];
        
       //This should be coherent with the behaviour of UndirectedTree
       if( root->second.children.size() != 1 || root_child->second.segment.getJoint().getType() != Joint::None )
       {  
           if( tree.getNrOfJoints() != dofs.size() || tree.getNrOfSegments()+1 !=  links.size() ) {
                return false;
           }
       } else {
           if( tree.getNrOfJoints() != dofs.size() || tree.getNrOfSegments() !=  links.size() ) {
                return false;
           }
       }
        
        unsigned int i;
        
        const SegmentMap & seg_map = tree.getSegments();
        
        for(i = 0; i < links.size(); i++ ) {
            seg = tree.getSegment(links[i]);
            if( seg == seg_map.end() ) {
                return false;
            }
        }
        
        for(SegmentMap::const_iterator it=seg_map.begin(); it != seg_map.end(); it++) {
            if( it->second.segment.getJoint().getType() != Joint::None ) {
                if( getJunctionId( it->second.segment.getJoint().getName()) == -1 ) {
                    return false;
                }
            }
        }
        
        return true;
        
    }
    
    int TreeSerialization::setNrOfLinks(const int new_size)
    {
        links.resize(new_size);
        if( new_size > 0 ) { 
            junctions.resize(new_size-1);
        } else {
            junctions.resize(0);
        }
        return links.size();
    }
            
    int TreeSerialization::getNrOfLinks() const
    {
        return links.size();
    }
    

    int TreeSerialization::getNrOfJunctions() const
    {
        return junctions.size();
    }
    
    int TreeSerialization::setNrOfDOFs(const int new_size)
    {
        dofs.resize(new_size);
        return dofs.size();
    }
    
    int TreeSerialization::getNrOfDOFs() const
    {
        return dofs.size();
    }
    
    bool TreeSerialization::serialize(const Tree & tree,
                                      std::vector< int> & children_root, //set of children of root
                                      std::vector< std::vector<int> > & children, //array of sets of children of each segment
                                      std::vector< int > & parent, //array of parent of each segment
                                      std::vector< int> & link2joint, //array mapping 
                                      std::vector< int > & forward_visit_order, //Visiting order for the tree, such that a parent is visited before any of his children
                                      std::vector<SegmentMap::const_iterator> & seg_vector //array of mapping between link index and SegmentMap iterators
                                      )
    {
        
        //assuming that *this and tree are consistent
        //the deprecated method is more efficient
        const SegmentMap& sm = tree.getSegments();
        
        children_root.resize(0);
        children.resize(tree.getNrOfSegments(),std::vector< int >(0));
        parent.resize(tree.getNrOfSegments());
        
        link2joint.resize(tree.getNrOfSegments(),FIXED_JOINT);
                
        seg_vector.resize(tree.getNrOfSegments());
        
        
        SegmentMap::const_iterator root, i;
        
        root = tree.getRootSegment();
        for( unsigned int j=0; j < root->second.children.size(); j++ ) {
            children_root.push_back(this->getLinkId(root->second.children[j]->first));
        }
        
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            if( i != root ) {
                unsigned int i_index = this->getLinkId(i->first);
                seg_vector[i_index] = i;
                
                for( unsigned int j=0; j < i->second.children.size(); j++ ) {
                    children[i_index].push_back(this->getLinkId(i->second.children[j]->first));
                }
                
                if( i->second.segment.getJoint().getType() != Joint::None ) {
                    link2joint[i_index] = this->getDOFId(i->second.segment.getJoint().getName());
                }
                
                if( i->second.parent == root ) {
                    parent[i_index] = -1;
                } else {
                    parent[i_index] = this->getLinkId(i->second.parent->first);
                }
                
            }
		}
        
        //Computing (only once, as given a root node and the structure 
        //the visiting order remains the same
        std::vector<unsigned int> index_stack;
        
        index_stack.reserve(tree.getNrOfSegments());
        forward_visit_order.reserve(tree.getNrOfSegments());
        
        index_stack.clear();
        forward_visit_order.clear();
        
        for( unsigned int j=0; j < children_root.size(); j++ ) {
            index_stack.push_back(children_root[j]);
        }
        
        while( !index_stack.empty() ) {
            
            unsigned int curr_index = index_stack.back();
            index_stack.pop_back();
            
            forward_visit_order.push_back(curr_index);
            
            //Doing the recursion on the children
            for( unsigned int j=0; j < children[curr_index].size(); j++ ) {
                index_stack.push_back(children[curr_index][j]);
            }
        }

        assert(forward_visit_order.size() == tree.getNrOfSegments());
        
        return true;
        
    }
    
    std::string TreeSerialization::toString()
    {
        std::stringstream ss;
        ss << "Links serialization:" << std::endl;
        for( int i = 0; i < (int)links.size(); i++ ) {
            ss <<  i << "\t: " << links[i] << std::endl;
        }
        ss << "Junctions serialization:" << std::endl;
        for( int i = 0; i < (int)junctions.size(); i++ ) {
            ss << i << "\t: " << junctions[i] << std::endl;
        }
        ss << "DOFs serialization:" << std::endl;
        for( int i = 0; i < (int)dofs.size(); i++ ) {
            ss << i << "\t: " << dofs[i] << std::endl;
        }
        return ss.str();
    }

}
}
