/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
  
#include "kdl_codyco/treeserialization.hpp"
#include <kdl/joint.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>

#include <kdl_codyco/utils.hpp>

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
        if( !isBaseLinkFake(tree) )
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
    
    int TreeSerialization::getJunctionID(std::string junction_name) const
    {
        std::vector<std::string>::const_iterator it;
        it = std::find(junctions.begin(),junctions.end(),junction_name);
        if( it != junctions.end() ) {
            return it - junctions.begin();
        } else {
            return -1;
        }
    }
        
    int TreeSerialization::getLinkID(std::string link_name) const
    {
        std::vector<std::string>::const_iterator it;
        it = std::find(links.begin(),links.end(),link_name);
        if( it != links.end() ) {
            return it - links.begin();
        } else {
            return -1;
        }
    }
    
    int TreeSerialization::getDOFID(std::string dof_name) const
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
    
    bool TreeSerialization::setDOFNameID(const std::string dof_name, const int new_ID)
    {
        if( new_ID < 0 || new_ID >= getNrOfDOFs() ) { return false; }
        dofs[new_ID] = dof_name;
        return true;
    }
    
    bool TreeSerialization::setJunctionNameID(const std::string junction_name, const int new_ID)
    {
        if( new_ID < 0 || new_ID >= getNrOfJunctions() ) { return false; }
        junctions[new_ID] = junction_name;
        return true;
    }
    
    bool TreeSerialization::setLinkNameID(const std::string link_name, const int new_ID)
    {
        if( new_ID < 0 || new_ID >= getNrOfLinks() ) { return false; }
        links[new_ID] = link_name;
        return true;
    }

    
    bool TreeSerialization::is_consistent(const Tree & tree) const
    {
        SegmentMap::const_iterator seg;
        
       SegmentMap::const_iterator root = tree.getRootSegment();
        
       /** \todo remove this assumption */
       assert(root->second.children.size() != 0);
       SegmentMap::const_iterator root_child = root->second.children[0];
        
       //This should be coherent with the behaviour of UndirectedTree
       if( !isBaseLinkFake(tree) )
       {  
           if( tree.getNrOfJoints() != dofs.size() || tree.getNrOfSegments()+1 !=  links.size() ) {
                std::cerr << "TreeSerialization::is_consistent returning false: because: tree is (dofs,links) " <<
                             tree.getNrOfJoints() << " " <<  tree.getNrOfSegments()+1 << " while serialization " <<
                             dofs.size() << " " << links.size() << std::endl;
                return false;
           }
       } else {
           if( tree.getNrOfJoints() != dofs.size() || tree.getNrOfSegments() !=  links.size() ) {
                 std::cerr << "TreeSerialization::is_consistent returning false: because: tree is (dofs,links) " <<
                             tree.getNrOfJoints() << " " <<  tree.getNrOfSegments() << " while serialization " <<
                             dofs.size() << " " << links.size() << std::endl;
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
                if( getJunctionID( it->second.segment.getJoint().getName()) == -1 ) {
                    return false;
                }
            }
        }
        
        return true;
        
    }
    
    int TreeSerialization::setNrOfLinks(const int new_size)
    {
        links.resize(new_size);
        return links.size();
    }
            
    int TreeSerialization::getNrOfLinks() const
    {
        return links.size();
    }
    
    int TreeSerialization::setNrOfJunctions(const int new_size)
    {
        junctions.resize(new_size);
        return junctions.size();
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
    
    bool TreeSerialization::loadLinksFromStringVector(const std::vector<std::string> & links_serialization)
    {
        if( links_serialization.size() != getNrOfLinks() ) {
            return false;
        }
        
        for(int i=0; i < getNrOfLinks(); i++ ) {
            setLinkNameID(links_serialization[i],i);
        }
        
        return true;
    }
        
    bool TreeSerialization::loadLinksFromFile(const std::string file_name)
    {
        std::vector<std::string> links_serialization(getNrOfLinks());
        
        if( !stringVectorFromFile(file_name,links_serialization,getNrOfLinks()) ) { return false; }

        
        return loadLinksFromStringVector(links_serialization);
    }
        
    bool TreeSerialization::loadJunctionsDOFsFromStringVector(const std::vector<std::string> & junctions_serialization)
    {
        if( junctions_serialization.size() != getNrOfJunctions() ) {
            return false;
        }
        
        for(int i=0; i < getNrOfJunctions(); i++ ) {
            if( i < getNrOfDOFs() ) {
                setDOFNameID(junctions_serialization[i],i);
            }
            setJunctionNameID(junctions_serialization[i],i);
        }
        
        return true;
    }
        
    bool TreeSerialization::loadJunctionsDOFsFromFile(const std::string file_name)
    {
        std::vector<std::string> junctions_serialization(getNrOfJunctions());
        
        if( !stringVectorFromFile(file_name,junctions_serialization,getNrOfJunctions()) ) { return false; }
        
        return loadJunctionsDOFsFromStringVector(junctions_serialization);
    }

}
}
