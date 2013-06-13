/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef KDL_CODYCO_TREE_ROTATION_HPP
#define KDL_CODYCO_TREE_ROTATION_HPP

#include <kdl/tree.hpp>


namespace KDL { 
namespace CoDyCo {
    
    bool addTreeRecursive(KDL::SegmentMap::const_iterator old_tree_root, KDL::Tree & new_tree, const std::string& hook_name);


    bool addParentRecursive(KDL::Tree & new_tree, KDL::SegmentMap::const_iterator old_child, const std::string& dummy_root_name);


    /**
     * The position of the Joint with respect to the hook segment is defined with respect
     * to the frame of reference of the segment. This function is used to
     * change the reference frame to another one (for example while rotating a tree)
     * 
     */
    KDL::Joint JointChangeReference( const KDL::Frame & frame, const KDL::Joint & old_joint);
    
    /**
     * Rotate a KDL::Tree to obtain an equivalent rapresentation, but with a different root link.
     *
     * To perform the rotation it is necessary that the first segment of the tree has a Joint::None joint.
     * 
     * @param old_tree the input KDL::Tree to rotate 
     * @param new_tree the output rotated KDL::Tree
     * @param new_root_name the name of the link to use as a new root
     * 
     * @return true if all went well, false otherwise
     * 
     * \todo add error checking
     */
    bool tree_rotation(const KDL::Tree & old_tree, KDL::Tree & new_tree, const std::string& new_root_name);

}
}
             
#endif
