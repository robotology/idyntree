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
    
    /**
     * Class for describing a Tree serialization (i.e. : a mapping between
     * the joints of the tree and 0 .. nrOfJoints-1 and 0 .. nrOfSegments-1)  
     * Please note that the joints "names" are actually the name of the segments
     * relative to the joint, so (in URDF terms) the relative link name.
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
        void addDFSrecursive(SegmentMap::const_iterator current_el,int & link_cnt);
        std::vector<std::string> links;
        std::vector<std::string> joints;
        
    public:
        TreeSerialization();
        /**
         * Constructor that builds the default serialization for KDL::Tree
         * For joints the one used to build the q_nr attribute in KDL::TreeElement
         * For links a DFS visit of the KDL tree is done 
         */
        TreeSerialization(const Tree & tree);
        
        TreeSerialization(std::vector<std::string> & links_in, std::vector<std::string> & joints_in);
        
        ~TreeSerialization();
        
        /**
         * Not efficient, performs a search
         */
        int getJointId(std::string joint_name);
        
        /**
         * Not efficient, performs a search
         */
        int getLinkId(std::string link_name);
        
        std::string getJointName(int joint_id);
        std::string getLinkName(int link_id);
        
        int getNrOfSegments();
        
        int getNrOfJoints();
        
        /**
         * Check if the TreeSerialization is a valid serialization for the 
         * given Tree
         * 
         */
        bool is_consistent(const Tree & tree);
        
        bool serialize(const Tree & tree,
                       std::vector< int> & mu_root, //set of childrens of root
                       std::vector< std::vector<int> > & mu, //array of sets of childrens of each segment
                       std::vector< int > & lambda, //array of parent of each segment
                       std::vector< int> & link2joint, //array mapping 
                       std::vector< int > & recursion_order, //Visiting order for the tree, such that a parent is visited before any of his childrens
                       std::vector<SegmentMap::const_iterator> & seg_vector //array of mapping between link index and SegmentMap iterators
                                         );
                                         
        std::string toString();
    };
    
    
}



#endif
