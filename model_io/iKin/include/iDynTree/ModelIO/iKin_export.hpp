/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Istituto Italiano di Tecnologia,
*   CoDyCo project
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
