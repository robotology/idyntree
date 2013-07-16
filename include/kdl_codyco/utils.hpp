/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_UTILS_HPP
#define KDL_CODYCO_UTILS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/treegraph.hpp>

namespace KDL {
namespace CoDyCo {

    /**
     * Compute the total mass of a KDL::Tree object
     * 
     * @param tree the KDL::Tree object
     * @return the mass of the robot, < 0 if something failed
     * 
     */
    double computeMass(const Tree & tree);
    
    
    /**
     * The position of the Joint with respect to the hook segment is defined with respect
     * to the frame of reference of the parent. This function is used to get the joint with the polarity
     * of the segment inverted.
     * 
     * \todo check the math in this function
     * 
     */
	int JointInvertPolarity(const KDL::Joint & old_joint, const KDL::Frame & old_f_tip, KDL::Joint & new_joint, KDL::Frame & new_f_tip);
    

}
}  



#endif 
