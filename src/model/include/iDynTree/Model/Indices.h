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

// Workaround for SWIG problems with GenerateExportHeader-generated code
#if defined(SWIG)
#define IDYNTREE_MODEL_EXPORT
#else
#include "ModelExport.h"
#endif

namespace iDynTree
{
    typedef std::ptrdiff_t LinkIndex;
    IDYNTREE_MODEL_EXPORT extern LinkIndex LINK_INVALID_INDEX;
    IDYNTREE_MODEL_EXPORT extern std::string LINK_INVALID_NAME;

    typedef std::ptrdiff_t JointIndex;
    IDYNTREE_MODEL_EXPORT extern std::ptrdiff_t JOINT_INVALID_INDEX;
    IDYNTREE_MODEL_EXPORT extern std::string JOINT_INVALID_NAME;

    typedef std::ptrdiff_t DOFIndex;
    IDYNTREE_MODEL_EXPORT extern std::ptrdiff_t DOF_INVALID_INDEX;
    IDYNTREE_MODEL_EXPORT extern std::string DOF_INVALID_NAME;

    typedef std::ptrdiff_t FrameIndex;
    IDYNTREE_MODEL_EXPORT extern std::ptrdiff_t FRAME_INVALID_INDEX;
    IDYNTREE_MODEL_EXPORT extern std::string FRAME_INVALID_NAME;

    typedef std::ptrdiff_t TraversalIndex;
    IDYNTREE_MODEL_EXPORT extern TraversalIndex TRAVERSAL_INVALID_INDEX;

}

#endif /* IDYNTREE_INDICES_H */
