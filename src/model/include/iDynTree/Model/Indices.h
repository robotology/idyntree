// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
