/**
 * Copyright  (C) 2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#ifndef KDL_TREE_PARTITION_HPP
#define KDL_TREE_PARTITION_HPP

#include <string>
#include <kdl/tree.hpp>
#include "kdl_codyco/treeserialization.hpp"

namespace KDL{
namespace CoDyCo {
    
    /**
     * Class describing a Part of a Tree
     * The class contains the unique integer id (in the tree ) of the part,
     * its name and the links and joints it contains. The serialization of joint
     * and links in the part is implicitly defined by the serialization of the
     * joints andl links in the tree by eliminating the joint and link
     * that do not belong to the part.
     */
    class TreePart{
        private:
            int part_id;
            std::string part_name;
            std::vector<int> dof_id;
            std::vector<int> links_id;
            
        public: 
            /**
             * Constructor use only for reporting error
             */
            TreePart();
            TreePart(int _part_id, std::string _part_name, std::vector<int> _dof_id, std::vector<int> _links_id);
            
            ~TreePart();
            
            int getNrOfLinks() const;
            int getNrOfDOFs() const;
            
            std::string getPartName() const;
            int getPartID() const;

    };
    
    
    /**
     * Class for describing a Tree partition (in the set theory sense)
     * (i.e. a scomposition of the Tree joints and links in non overlapping 
     * sets).
     * 
     */
    class TreePartition {
    private:
        std::vector<TreePart> parts;
        std::map<std::string,int> map_part_id;
        
    public:
        TreePartition(); 
        
        ~TreePartition();
         
        /**
         * Add a part to a given partition, parts are required to be 
         * added in the right order, consistent with their ID
         * 
         * @return true if all went well, false otherwise
         */
        bool addPart(TreePart & tree_part);
        
        TreePart getPart(int id);
        TreePart getPart(std::string part_name); 
         
        /**
         * Check if the TreePartition is a valid serialization for the 
         * given Tree (and optionally the given TreeSerialization ) 
         * 
         */
        bool is_consistent(const Tree & tree ) const;
        bool is_consistent(const Tree & tree, TreeSerialization  tree_serialization) const;
               
        std::string toString() const;
    };
    
}
    
}

#endif
