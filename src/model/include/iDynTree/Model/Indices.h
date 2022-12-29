/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INDICES_H
#define IDYNTREE_INDICES_H

#include <cstddef>
#include <string>

namespace iDynTree
{
    typedef std::ptrdiff_t LinkIndex;
    constexpr LinkIndex LINK_INVALID_INDEX = -1;
    constexpr char[] LINK_INVALID_NAME = "";

    typedef std::ptrdiff_t JointIndex;
    constexpr std::ptrdiff_t JOINT_INVALID_INDEX = -1;
    constexpr char[] JOINT_INVALID_NAME = "";

    typedef std::ptrdiff_t DOFIndex;
    constexpr std::ptrdiff_t DOF_INVALID_INDEX = -1;
    constexpr char[] DOF_INVALID_NAME = "";

    typedef std::ptrdiff_t FrameIndex;
    constexpr std::ptrdiff_t FRAME_INVALID_INDEX = -1;
    constexpr char[] FRAME_INVALID_NAME = "";

    typedef std::ptrdiff_t TraversalIndex;
    constexpr TraversalIndex TRAVERSAL_INVALID_INDEX = -1;

}

#endif /* IDYNTREE_INDICES_H */
