/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#ifndef __ICUBTREE_H__
#define __ICUBTREE_H__

#include <iCub/iDynTree/DynTree.h>

#include <iostream>

namespace iCub
{

namespace iDynTree
{

const std::string ICUB_IMU_LINK_NAME = "imu_frame";  
  
/**
 * Struct for describing the version of the parts of an iCubTree
 */
struct iCubTree_version_tag
{
    int head_version; /**< Head version: can be 1 or 2 (default: 1) */ 
    int legs_version; /**< Legs version: can be 1 or 2 (default: 1) */
    bool feet_ft; /**< Feet ft: can be true (structure with FT) or false (default: true) */
    
    iCubTree_version_tag () 
    {
        head_version=1;
        legs_version=1;
        feet_ft=true;
    }
    
    iCubTree_version_tag (int _head_version, int _legs_version, bool _feet_ft) 
    {
        head_version=_head_version;
        legs_version=_legs_version;
        feet_ft=_feet_ft;
    }
  
  
};

/**
 * Enum for describing the serialization used for iCubTree
 */
enum iCubTree_serialization_tag
{
    IDYN_SERIALIZATION, /**< The serialization used in iCubWholeBody:
                             left leg (6), right leg (6), torso (3), left arm (7), right arm (7), head (3) (Jorhabib) */
    SKINDYNLIB_SERIALIZATION /**< The serialization (implicitly) used in skinDynLib: 
                              torso(3), head (3), left arm (7), right arm(7), left leg (6), right leg (6) (Andrea) */
};

/**
 *  \ingroup iDynTree
 * 
 * Class that instantiate a DynTree object with the model of the iCub robot
 */
class iCubTree : public DynTree 
{
    private:
        /**
         * Get the partition of the iCub in a skinDynLib   
         * 
         */
        KDL::CoDyCo::TreePartition get_iCub_partition(const KDL::CoDyCo::TreeSerialization & icub_serialization, bool ft_feet);

    public:
        
    /**
     * Constructor for iCubTree
     * 
     * \note the FT sensor serialization is (0) LEFT_ARM (1) RIGHT_ARM (2) LEFT_LEG (3) RIGHT_LEG
     * \note currently the iCub model is loaded from iCub::iDyn::iCubWholeBody. This is a temporary workaround for initial deployment, 
     *       while in the final version of iDynTree the structure of the iCub should be loaded from a file description.
     *       
     * @param version a iCubTree_version_tag structure for defining the version of the parts composing the iCubTree
     * @param serial a iCubTree_serialization_tag for defining the serialization (default is SKINDYNLIB_SERIALIZATION)
     * @param verbose level of verbosity: 0 if no output is requested, 1 to have output messages (default is 0) 
     * @param imu_link_name name of the link to consider as imu (default: "imu_link")
     */
     iCubTree(iCubTree_version_tag version, iCubTree_serialization_tag serial=SKINDYNLIB_SERIALIZATION,  unsigned int verbose=0, std::string imu_link_name = ICUB_IMU_LINK_NAME  );

    virtual ~iCubTree();
};

}//end namespace

}

#endif
