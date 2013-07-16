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
     * 
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
            TreePart(int _part_id, std::string _part_name);
            TreePart(int _part_id, std::string _part_name, std::vector<int> _dof_id, std::vector<int> _links_id);
            
            void addDOF(int global_dof_index) { dof_id.push_back(global_dof_index); }
            void addLink(int global_link_index) { links_id.push_back(global_link_index); };
            
            ~TreePart();
            
            int getNrOfLinks() const;
            int getNrOfDOFs() const;
            
            std::string getPartName() const;
            int getPartID() const;
            
            int getGlobalLinkIndex(int local_link_index) const { return links_id[local_link_index]; }
            int getGlobalDOFIndex(int local_dof_index) const { return dof_id[local_dof_index]; }
            
            const std::vector<int> & getLinkIDs() const { return links_id; }
			const  std::vector<int> & getDOFIDs() const { return dof_id; }

    };
    
    
    /**
     * Class for describing a Tree partition (in the set theory sense)
     * (i.e. a scomposition of the Tree DOFs and links in non overlapping 
     * sets).
     * 
     * The ID of a part can be any positive number. A part is composed by 
     * a (numbered) list of serialized DOFs and a (numbered) list of serialiazed links.
     * 
     * To properly initialize a TreePartition, define before the TreePart object
     * and add them by using the addPart method
     * 
     * \note This class is not optimized, as it is used only in the inizialization
     *       of the solvers.
     * 
     */
    class TreePartition {
    private:
        //Vector of parts
        std::vector<TreePart> parts;
        
        //Map from the part name to the index of the TreePart in vector parts
        std::map<std::string,int> name_map;
        
        //Map from the part ID to the index of the TreePart in vector parts
        std::map<int,int> ID_map;
        
        int getLocalPartIndex(const std::string part_name);
        
    public:
        /**
         * Constructor
         */
        TreePartition(); 
        
        TreePartition(const Tree & tree);
        
        /**
         * Destructor
         */
        ~TreePartition();
         
        int getNrOfParts() const { return parts.size(); } ;
        
        /**
         * Get the part from its local index in this partition 
         * (the local index is different from the part ID, that can be
         *  an arbitrary positive number, while the local index 
         *  is a number between 0 and NrOfParts-1
         */
        TreePart getPartFromLocalIndex(int part_index)
        {
			assert( part_index >= 0 && part_index < getNrOfParts() );
			return parts[part_index];
		}
        
        /**
         * Add a part to a given partition
         * 
         * @return true if all went well, false otherwise
         */
        bool addPart(TreePart & tree_part);
        
        /**
         * Get a part object by the ID
         * @return the correct TreePart is all went well, otherwise an empty one
         */
        TreePart getPart(int part_ID) const;
        
        /**
         * Get a part object by the name
         * @return the correct TreePart is all went well, otherwise an empty one
         */
        TreePart getPart(std::string part_name) const; 
        
        /**
         * Get the part ID from the global link ID
         * @param global_link_index the global ID for the link
         * 
         */
        int getPartIDfromLink(int global_link_index) const;
        
         /**
         * Get the part ID from the global DOF ID
         * @param global_link_index the global ID for the degree of freedom
         * @return the part ID
         */
        int getPartIDfromDOF(int global_DOF_index) const;
        
        /**
         * Get the global link ID from the body part ID and the local link ID
         */
        int getGlobalLinkIndex(int part_ID, int local_link_index) const;
        
        /**
         * Get the global DOF ID from the couple of body part ID and local DOF ID
         */
        int getGlobalDOFIndex(int part_ID, int local_DOF_index) const;
        
        /**
         * Get the local link ID from the global link ID
         */
        int getLocalLinkIndex(int global_link_index) const;
        
        /**
         * Get the local DOF id from the global dof ID
         */
        int getLocalDOFIndex(int global_dof_index) const;
        
        const std::vector<int> & getPartLinkIDs(std::string part_name) const;
        
        const std::vector<int> & getPartDOFIDs(std::string part_name) const;

         
        /**
         * Check if the TreePartition is a valid serialization for the 
         * given Tree (and optionally the given TreeSerialization ) 
         * 
         */
        bool is_consistent(const Tree & tree ) const;
        bool is_consistent(const Tree & tree, TreeSerialization  tree_serialization) const;
        
        /**
         * Print to a string the TreePartition (for debug purpose)
         */
        std::string toString() const;
    };
    
}
    
}

#endif
