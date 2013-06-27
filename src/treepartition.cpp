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
    TreePart::TreePart()
    {
        part_id = -1;
        part_name = "TreePartError";
    }
    
    TreePart::TreePart(int _part_id, std::string _part_name, std::vector<int> _dof_id, std::vector<int> _links_id): part_id(_part_id), part_name(_part_name), dof_id(_dof_id), links_id(_links_id)
    {
    }
    
    TreePart::~TreePart()
    {
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
    
    TreePartition::TreePartition()
    {
    } 
        
    TreePartition::~TreePartition()
    {
    }
         
    bool TreePartition::addPart(TreePart & tree_part)
    {
        if( tree_part.getPartID() != (int)parts.size() ) {
            return false;
        }
        //else
        parts.push_back(tree_part);
        return true;
    }
        
    TreePart TreePartition::getPart(int id)
    {
        if( id < 0 || id >= (int)parts.size() ) {
            return TreePart();
        }
        
        return parts[id];
    }
    
    TreePart TreePartition::getPart(std::string part_name)
    {
        int id;
        
        std::map<std::string,int>::const_iterator it = map_part_id.find(part_name);
        
        if( it == map_part_id.end() ) {
            return TreePart();
        }
        
        id = it->second;
        
        return getPart(id);
    }
         
    /**
    * Check if the TreePartition is a valid serialization for the 
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
            if( part.getPartID() != i ) return false;
            if( part.getNrOfLinks() <= 0 ) return false;
            total_links +=  part.getNrOfLinks();
        }
        
        if( total_links != (int)tree.getNrOfSegments() ) return false;
        
        /**
         * \todo add link/joint id level serialization
         */
        
        return true;
    }
        
    std::string TreePartition::toString() const
    {
        return "";
    }
}
}
