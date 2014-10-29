/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#ifndef ICUBTREE_H
#define ICUBTREE_H

#include <iCub/iDynTree/DynTree.h>

#include <iostream>

namespace iCub
{

namespace iDynTree
{
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
     *
     * @param urdf_filename
     * @param verbose
     */
    iCubTree(std::string urdf_filename, std::string fixed_link="", unsigned int verbose=0);

    virtual ~iCubTree();
};

}//end namespace

}

#endif
