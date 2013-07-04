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
    
}
}  



#endif 
