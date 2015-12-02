/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNAMICS_UTILS_H
#define IDYNTREE_DYNAMICS_UTILS_H

#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/SpatialInertia.h>

namespace iDynTree
{
    /**
     * Given a rigid body inertia \f$M\f$ and spatial motion vector \f$V\f$,
     * the bias wrench \f$B\f$ of rigid body is defined as:
     * \f[
     *   B =  \cross
     * \f]
     *
     */
    biasWrenchVelocityDerivative(SpatialInertia M, SpatialMotionVector V);
}


#endif /* IDYNTREE_DYNAMICS_UTILS_H */