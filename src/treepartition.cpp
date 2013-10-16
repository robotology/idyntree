/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
  
#include "kdl_codyco/treepartition.hpp"
#include <kdl/joint.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>

namespace KDL {
namespace CoDyCo {
    /*
    int TreePart::getLocalPartIndex(const std::string part_name)
    { 
        std::map<std::string,int>::const_iterator it_si = name_map.find(part_name);
        
        if( it_si == name_map.end() ) { return -1; }
        
        int part_id = it_si->second;
        
                
        std::map<int,int>::const_iterator it_ii = ID_map.find(part_id);
        
        if( it_ii == ID_map.end() ) { return -1; }
        
        int part_local_index = it_ii->second;
        
        return part_local_index;
	}*/
    
    TreePart::TreePart(): part_id(-1), part_name("TreePartError"), dof_id(0), links_id(0)
    {
    }
    
    TreePart::TreePart(int _part_id, std::string _part_name): part_id(_part_id), part_name(_part_name), dof_id(0), links_id(0)
    {
    }
    
    TreePart::TreePart(int _part_id, std::string _part_name, std::vector<int> _dof_id, std::vector<int> _links_id): part_id(_part_id), part_name(_part_name), dof_id(_dof_id), links_id(_links_id)
    {
    }
    
    TreePart::~TreePart()
    {
    }
     
    TreePart& TreePart::operator=(const TreePart& x) 
	{
		if ( this != &x ) { 
			this->part_id = x.part_id;
			this->part_name = x.part_name;
			this->dof_id = x.dof_id;
			this->links_id = x.links_id;
		}
		return *this;
	}
           
    int TreePart::getNrOfLinks() const
    {
        return links_id.size();
    }
    
    int TreePart::getNrOfDOFs() const
    {
        return dof_id.size();
    }
            
    std::string TreePart::getPartName() const
    {
        return part_name;
    }
    
    int TreePart::getPartID() const
    {
        return part_id;
    }
    
    std::string TreePart::toString() const
    {
		std::stringstream ss;
		ss << "TreePart: " << part_id << " " << part_name << std::endl;
        return ss.str();
    }
    
    TreePartition::TreePartition()
    {
    } 
        
    TreePartition::~TreePartition()
    {
    }
    
    TreePartition::TreePartition(const Tree & tree)
    {
        int nrOfDOFs = tree.getNrOfJoints();
        int nrOfLinks = tree.getNrOfSegments();
        
        TreePart tree_part(0,"default_part");
        
        SegmentMap::const_iterator root = tree.getRootSegment();
        
        /** \todo remove this assumption */
        assert(root->second.children.size() != 0);
        SegmentMap::const_iterator root_child = root->second.children[0];
        
         //This should be coherent with the behaviour of UndirectedTree
        if( root->second.children.size() != 1 || root_child->second.segment.getJoint().getType() != Joint::None )
        {
            nrOfLinks++;
        } 
        
        for(int i=0; i <nrOfDOFs; i++ )
        {
            tree_part.addDOF(i);
        }
        
        for(int i=0; i <nrOfLinks; i++ )
        {
            tree_part.addLink(i);
        }
        
        addPart(tree_part);
    }

    bool TreePartition::addPart(TreePart & tree_part)
    {
		int part_index = parts.size();
        parts.push_back(tree_part);
        ID_map.insert(std::make_pair(tree_part.getPartID(),part_index));
        name_map.insert(std::make_pair(tree_part.getPartName(),part_index));
        return true;
    }
        
    TreePart TreePartition::getPart(int id) const
    {
        int local_index;
        
        std::map<int,int>::const_iterator it = ID_map.find(id);
        
        if( it == ID_map.end() ) {
            return TreePart();
        }
        
        local_index = it->second;
        
        return parts[local_index];
    }
    
    TreePart TreePartition::getPart(const std::string & part_name) const
    {
        int local_index;
        
        std::map<std::string,int>::const_iterator it = name_map.find(part_name);
        
        if( it == name_map.end() ) {
            return TreePart();
        }
        
        local_index = it->second;
        
        return parts[local_index];
    }
    
    int TreePartition::getPartIDfromPartName(const std::string & part_name) const
    {
       int local_index;
        
        std::map<std::string,int>::const_iterator it = name_map.find(part_name);
        
        if( it == name_map.end() ) {
            return -1;
        }
        
        local_index = it->second;
        
        return parts[local_index].getPartID();
    }
    
    int TreePartition::getPartIDfromLink(int link_id) const
    {
        for(int i=0; i < (int)parts.size(); i++ ) {
            for( int j=0; j < parts[i].getNrOfLinks(); j++ ) {
                if( parts[i].getGlobalLinkIndex(j) == link_id ) {
                    return parts[i].getPartID();
                }
            }
        }
        return -1;
    }
    
    int TreePartition::getPartIDfromDOF(int dof_id) const
    {
        for(int i=0; i < (int)parts.size(); i++ ) {
            for( int j=0; j < parts[i].getNrOfDOFs(); j++ ) {
                if( parts[i].getGlobalDOFIndex(j) == dof_id ) {
                    return parts[i].getPartID();
                }
            }
        }
        return -1;
    }

    int TreePartition::getGlobalLinkIndex(int part_ID, int local_link_index) const
    {    
        int local_index;
        
        std::map<int,int>::const_iterator it = ID_map.find(part_ID);
        
        if( it == ID_map.end() ) {
            return -1;
        }
        
        local_index = it->second;
        
        if( local_link_index >= parts[local_index].getNrOfLinks() || local_link_index < 0 ) return -1;
        
        return parts[local_index].getGlobalLinkIndex(local_link_index);
    }
        
    int TreePartition::getGlobalDOFIndex(int part_ID, int local_DOF_index) const
    {
        int local_index;
        
        std::map<int,int>::const_iterator it = ID_map.find(part_ID);
        
        if( it == ID_map.end() ) {
            return -1;
        }
        
        local_index = it->second;
        
        return parts[local_index].getGlobalDOFIndex(local_DOF_index);
    }
        
    int TreePartition::getLocalLinkIndex(int global_link_index) const
    {
        for(int i=0; i < (int)parts.size(); i++ ) {
            for( int j=0; j < parts[i].getNrOfLinks(); j++ ) {
                if( parts[i].getGlobalLinkIndex(j) == global_link_index ) {
                    return j;
                }
            }
        }
        return -1;
    }
        
    int TreePartition::getLocalDOFIndex(int global_dof_index) const
    {
        for(int i=0; i < (int)parts.size(); i++ ) {
            for( int j=0; j < parts[i].getNrOfDOFs(); j++ ) {
                if( parts[i].getGlobalDOFIndex(j) == global_dof_index) {
                    return j;
                }
            }
        }
        return -1;
    }
    
         
    /**
    * Check if the TreePartition is a valid partition for the 
    * given Tree (and optionally the given TreeSerialization ) 
    * 
    */
    bool TreePartition::is_consistent(const Tree & tree ) const
    {
        return is_consistent(tree,TreeSerialization(tree));
    }
    
    bool TreePartition::is_consistent(const Tree & tree, TreeSerialization tree_serialization) const
    {
        if( !tree_serialization.is_consistent(tree) ) return false;
        
        int total_links = 0;
        
        for(int i=0; i < (int)parts.size(); i++ ) {
            TreePart part = parts[i];
            if( part.getNrOfLinks() <= 0 ) return false;
            total_links +=  part.getNrOfLinks();
        }
        
        if( total_links != (int)tree.getNrOfSegments() ) return false;
        
        if( ID_map.size() != parts.size() ) { return false; }
        if( name_map.size() != parts.size() ) { return false; }

        
        /**
         * \todo add link/joint id level serialization
         */
        
        return true;
    }
    
    /**
     * \todo add efficient way of returning vector<int>
     * 
     */
    std::vector<int> TreePartition::getPartLinkIDs(const std::string & part_name) const
    {   
        //\todo error checking
        TreePart part = getPart(part_name);
        return part.getLinkIDs();
    }

    /**
     * \todo add efficient way of returning vector<int>
     * 
     */
    std::vector<int> TreePartition::getPartDOFIDs(const std::string & part_name) const
    {   
        std::vector<int> ret;
        #ifndef NDEBUG
        //std::cout << "getPartDOFIDs(" << part_name <<")" << std::endl;
        #endif 
        TreePart part = getPart(part_name);
        ret = part.getDOFIDs();
        #ifndef NDEBUG
        //std::cout << part.toString() << std::endl;
        //std::cout << "has " << ret.size() << " DOFs " << std::endl;
        #endif
        return ret;
    }

        
    std::string TreePartition::toString() const
    {
        std::stringstream ss;
        for(int i=0; i < (int)parts.size(); i++ ) {
            //std::cout << "TreePartition::toString() index " << i << " " << parts[i].getPartID() << std::endl;
            ss << "part ID:" << parts[i].getPartID() << " part name: " << parts[i].getPartName() << std::endl;
        }
        return ss.str();
    }
}
}
