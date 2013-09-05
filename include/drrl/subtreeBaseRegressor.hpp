/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */

#ifndef _DRRL_ARTICULATED_DYNAMICS_REGRESSOR_
#define _DRRL_ARTICULATED_DYNAMICS_REGRESSOR_

namespace DRRL 
{

class subtreeArticulatedDynamicsRegressor : public DynamicRegressorInterface 
{
    std::vector< std::string > subtree_leaf_links;
    std::vector< int > subtree_leaf_links_indeces;
    
    public:
        /**
         * 
         * @
         */
        subtreeArticulatedDynamicsRegressor(std::vector< std::string> _subtree_leaf_links=std::vector< std::string>(0)):subtree_leaf_links(_subtree_leaf_links) {};
        ~subtreeArticulatedDynamicsRegressor() {};
    
}

}
#endif
