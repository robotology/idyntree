/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN2KDL_ICUB_H 
#define IDYN2KDL_ICUB_H 

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <kdl/tree.hpp>
#include <iCub/iDynTree/iDyn2KDL.h>
#include <iCub/iDynTree/iCubTree.h>

/**
 * Get a KDL::Tree model from a iCub::iDyn::iCubWholeBody object
 * 
 * @param icub_idyn the iCub::iDyn::iCubWholeBody iCub input object
 * @param icub_kdl the iCub KDL::Tree output object
 * @param q_min a KDL::JntArray containing the joint minimum values (as KDL::Tree doesn't support joint limits) 
 * @param q_max a KDL::JntArray containing the joint minimum values (as KDL::Tree doesn't support joint limits)
 * @param serial a iCub::iDynTree::iCubTree_serialization_tag for specifing the serialization for the iCub (default: iCub::iDynTree::SKINDYNLIB_SERIALIZATION)
 * @param debug if true, add some frame useful for debug (default: false) 
 * @return false in case of error, true otherwise
 */
bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, KDL::JntArray & q_min, KDL::JntArray & q_max, iCub::iDynTree::iCubTree_serialization_tag serial=iCub::iDynTree::SKINDYNLIB_SERIALIZATION, bool ft_foot=false, bool add_root_weight=false, bool debug=false);

//bool toKDL_iDynDebug(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, bool debug=false);


#endif
