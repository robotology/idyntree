// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_JACOBIANS_H
#define IDYNTREE_JACOBIANS_H

#include <iDynTree/Indices.h>

namespace iDynTree
{
    class Model;
    class Traversal;
    class Transform;
    class FreeFloatingPos;
    class FreeFloatingVel;
    class FreeFloatingAcc;
    class LinkPositions;
    class LinkVelArray;
    class LinkAccArray;
    class JointPosDoubleArray;
    class MatrixDynSize;

    template<typename>
    class MatrixView;

    /**
     * \ingroup iDynTreeModel
     *
     * Compute a free floating jacobian
     *
     * @param[in]  model the used model,
     * @param[in]  traversal the used traversal,
     * @param[in]  jointPositions the vector of (internal) joint positions,
     * @param[in]  linkPositions linkPositions(l) contains the world_H_link transform.
     * @param[in]  linkIndex     the index of the link of which we compute the jacobian.
     * @param[in]  jacobFrame_X_world TODO
     * @param[in]  baseFrame_X_jacobBaseFrame TODO
     * @param[out] jacobian the computed Jacobian
     * @return true if all went well, false otherwise.
     */
    bool FreeFloatingJacobianUsingLinkPos(const Model& model,
                                          const Traversal& traversal,
                                          const JointPosDoubleArray& jointPositions,
                                          const LinkPositions& linkPositions,
                                          const LinkIndex linkIndex,
                                          const Transform & jacobFrame_X_world,
                                          const Transform & baseFrame_X_jacobBaseFrame,
                                          const MatrixView<double>& jacobian);


}

#endif
