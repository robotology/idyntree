/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#ifndef KDL_TREE_SERIALIZATION_HPP
#define KDL_TREE_SERIALIZATION_HPP

#include <string>
#include <kdl/tree.hpp>

namespace KDL{
namespace CoDyCo{
        
    /**
     * Class for describing a Tree serialization
     * (i.e. a mapping between:
     *      * Links and 0,1,...,nrOfLinks-1
     *      * DOFs and 0,1,...,nrOfJoints-1
     *      * Junction and 0,1,...,nrOfLinks-2 )
     * 
     * Please note that currently for simplicity is assumed that the Junctions with a DOF
     * have the same id of their DOF (then all the fixed junctions have higher IDs)
     * 
     * Assuming (as currently in KDL) that joints can have only 0 (fixed) or 1 dof.
     * 
     * A serialization can be considered valid also if the id of an segment
     * is not always lower then the id of any of its children 
     * 
     * \todo : in solver inizialization, getLinkName is used for all the links
     *         should be the case to replace the vector with a more efficient (in query)
     *          data structure?
     * 
     */
    class TreeSerialization {
    private:
        void addDFSrecursive(SegmentMap::const_iterator current_el,int & link_cnt, int & fixed_joint_cnt);

        void addDFSrecursive_only_fixed(SegmentMap::const_iterator current_el, int & fixed_joint_cnt);

        
    public:
        std::vector<std::string> links;
        std::vector<std::string> junctions;
        std::vector<std::string> dofs;
    
        TreeSerialization();
        
        /**
         * Constructor that builds the default serialization for KDL::Tree
         * For DOFs the one used to build the q_nr attribute in KDL::TreeElement
         * For links a DFS visit of the KDL tree is done
         * For joints the 1DOF joints have the same ID of their DOF, otherwise they are assigned a new ID higher then getNrOfDOFs
         */
        TreeSerialization(const Tree & tree);
        
        /**
         * Constructor that build the serialization for KDL::Tree
         * The link serialization is defined by the passed links_in vector
         * 
         * The DOF and joints serialization is defined by the joints_in vector: 
         * depending on its length it is considered as the vector of DOF or the vector of Junctions
         * In either cases it remains the constraint that the fixed junction have the higher IDs
         */
        TreeSerialization(const Tree & tree, std::vector<std::string> & links_in, std::vector<std::string> & joints_in);
        
        ~TreeSerialization();
        
        TreeSerialization(const TreeSerialization& x);
        
        /**
         * Not efficient, performs a search
         */
        int getJunctionId(std::string joint_name) const;
        
        /**
         * Not efficient, performs a search
         */
        int getDOFId(std::string dof_name) const;
        
        /**
         * Not efficient, performs a search
         */
        int getLinkId(std::string link_name) const;
        
        std::string getJunctionName(int joint_id) const;
        
        std::string getDOFName(int dof_id) const;
        
        std::string getLinkName(int link_id) const;
        
        
        /**
         * Set the number of Links
         */
        int setNrOfLinks(const int new_size);
        
        /**
         * Get the number of Links
         */
        int getNrOfLinks() const;
        
        /**
         * Get the number of internal degrees of freedom
         */
        int setNrOfDOFs(const int new_size);
        
        /**
         * Get the number of internal degrees of freedom
         */
        int getNrOfDOFs() const;
        
        /**
         * Get the number of joints of any DOF (not called getNrOfJoints to 
         *   avoid confusion with the function of KDL::Tree/KDL::Chain
         */
        int getNrOfJunctions() const;
        
        /**
         * Check if the TreeSerialization is a valid serialization for the 
         * given Tree (checking also if the names are the wright one, not 
         * if the DOF serialization inside the TreeSerialization is the same
         * of the one in the Tree
         * 
         */
        bool is_consistent(const Tree & tree) const;
              
        std::string toString();
    };
    
    
}
}


#endif
