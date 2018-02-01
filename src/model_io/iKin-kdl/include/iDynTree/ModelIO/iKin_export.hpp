/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/* Author: Silvio Traversaro */

#ifndef KDL_IKIN_EXPORT_H
#define KDL_IKIN_EXPORT_H

#include <kdl/tree.hpp>
#include <kdl/joint.hpp>
#include <kdl/jntarray.hpp>

#include <string>
#include <iCub/iKin/iKinFwd.h>

#include <iostream>


namespace iDynTree
{


/**
 * \ingroup iDynTreeModelIO
 *
 * Constructs a iKin limb model from a KDL Chain
 *  \note iKinLimb supports only rotational joints
 *        links connected by fixed joints are wielded
 *        if a translational joint is found an error is returned
 *
 * \param kdl_chain The KDL::Chain
 * \param iKin_chain The resulting iKinChain
 * \param min (optional) array of minimum values for joint position
 * \param max (optional) array of maximum values for joint position
 * @param verbose Verbosity level, default 0
 * returns true on success, false on failure
 */
bool iKinLimbFromKDLChain(const KDL::Chain& tree,
                          iCub::iKin::iKinLimb& iKin_limb,
                          const KDL::JntArray & min,
                          const KDL::JntArray & max,
                          int verbose=0);

}

#endif
