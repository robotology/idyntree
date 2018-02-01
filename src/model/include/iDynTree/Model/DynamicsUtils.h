/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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