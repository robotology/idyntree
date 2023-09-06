// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_DYNAMICS_UTILS_H
#define IDYNTREE_DYNAMICS_UTILS_H

#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/SpatialForceVector.h>
#include <iDynTree/SpatialInertia.h>

namespace iDynTree
{
    /**
     * Given a rigid body inertia \f$M\f$ and spatial motion vector \f$V\f$,
     * the bias wrench \f$B\f$ of rigid body is defined as:
     * \f[
     *   B =  \times
     * \f]
     *
     */
    biasWrenchVelocityDerivative(SpatialInertia M, SpatialMotionVector V);
}


#endif /* IDYNTREE_DYNAMICS_UTILS_H */