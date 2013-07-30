#include "kdl_codyco/tree_rotation.hpp"

using namespace KDL;


namespace KDL {
namespace CoDyCo {
    
bool addTreeRecursive(SegmentMap::const_iterator old_tree_root, Tree & new_tree, const std::string& hook_name)
{
    //get iterator for root-segment
    SegmentMap::const_iterator child;
    //try to add all of root's children
    for (unsigned int i = 0; i < old_tree_root->second.children.size(); i++) {
        child = old_tree_root->second.children[i];
        //Try to add the child
        if (new_tree.addSegment(child->second.segment, hook_name)) {
            //if child is added, add all the child's children
            if (!addTreeRecursive(child, new_tree, child->first))
                //if it didn't work, return false
                return false;
        } else
            //If the child could not be added, return false
            return false;
    }
    return true;
}

    
bool addParentRecursive(Tree & new_tree, SegmentMap::const_iterator old_child, const std::string& dummy_root_name)
{
    unsigned int i;
    //The parent of the new root is now a children
    SegmentMap::const_iterator old_parent = old_child->second.parent;
    
    new_tree.addSegment(Segment(old_parent->first,
                                JointChangeReference(old_child->second.segment.getFrameToTip().Inverse(),
                                                    old_child->second.segment.getJoint()),
                                old_child->second.segment.getFrameToTip().Inverse(),
                                old_parent->second.segment.getInertia()),
                        old_child->first);
                        
    //The children of the parent mantain the same structure, while recursivle the parent becomes the children
    //add all the children, besides the one already added
    for(i = 0; i < old_parent->second.children.size(); i++ ) {
        //Do not add the new parent
        if( old_parent->second.children[i]->first != old_child->first ) {
            SegmentMap::const_iterator child = old_parent->second.children[i];
            //Try to add the child
            if (new_tree.addSegment(child->second.segment, old_parent->first)) {
                //if child is added, add all the child's children
                if (!addTreeRecursive(child, new_tree, child->first))
                    //if it didn't work, return false
                    return false;
            } else
            //If the child could not be added, return false
            return false;
        }   
    }
    
    //Then the grandparent is added in a recursive way
    //Terminating condition: do not add the fake base link
    if( old_parent->second.parent->first != dummy_root_name ) 
        if( !addParentRecursive(new_tree,old_parent,dummy_root_name) ) 
            return false;
    return true;
}
    
    
    
    Joint JointChangeReference(const Frame & frame, const Joint & old_joint)
    {
        Joint new_joint;
        switch(old_joint.getType()) 
        {
            case Joint::RotAxis:
            case Joint::RotX:
            case Joint::RotY:
            case Joint::RotZ:
                new_joint = Joint(old_joint.getName(), frame*old_joint.JointOrigin(), frame.M*old_joint.JointAxis(), Joint::RotAxis);
            break;
            case Joint::TransAxis:
            case Joint::TransX:
            case Joint::TransY:
            case Joint::TransZ:
                new_joint = Joint(old_joint.getName(), frame*old_joint.JointOrigin(), frame.M*old_joint.JointAxis(), Joint::TransAxis);
            break;
            case Joint::None:
            default:
                new_joint = Joint(Joint::None);
            break;
        }
        return new_joint;
    }

    bool tree_rotation(const KDL::Tree & old_tree, KDL::Tree & new_tree, const std::string& new_root_name) 
    {
        SegmentMap::const_iterator dummy_root;
        SegmentMap::const_iterator old_root; //old *real* root 
        SegmentMap::const_iterator new_root; //new *real* root
        SegmentMap segments;

        
        segments = old_tree.getSegments();
        new_root = segments.find(new_root_name);
        //check if new root exists
        if (new_root == segments.end())
            return false;
        
        //check that the tree has a root segment with only one child, connected 
        //with a Joint::None (a fixed joint) (standard in humanoids URDF)
        dummy_root = old_tree.getRootSegment();
        if( dummy_root->second.children.size() != 1 ) 
            return false;
            
        old_root = dummy_root->second.children[0];
        if( old_root->second.segment.getJoint().getType() != Joint::None )
            return false;
        
        //The dummy_root is moved 
        new_tree = KDL::Tree(dummy_root->first);
        
        //the new root is the specified segment, but with Joint::None as the joint
        new_tree.addSegment(Segment(new_root_name,
                                    Joint(Joint::None),
                                    Frame::Identity(),
                                    new_root->second.segment.getInertia()),
                            dummy_root->first);
                            
        //Now that the new root is inserted, the successor subchain can be simply added
        //Not efficient, but to avoid code duplication
        //addTreeRecursive method is private, a copy is done
        if( !addTreeRecursive(new_root,new_tree,new_root->first) ) 
            return false;
    
        
        //Now the other part of the tree has to be added 
        return addParentRecursive(new_tree,new_root,dummy_root->first);
        
        
    }
        
}
}
