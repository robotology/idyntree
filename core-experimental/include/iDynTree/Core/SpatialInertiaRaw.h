/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_INERTIA_RAW_H
#define IDYNTREE_SPATIAL_INERTIA_RAW_H

#include "RotationalInertiaRaw.h"

namespace iDynTree
{
    class PositionRaw;

    /**
     * Class providing the raw coordinates for a spatial inertia, i.e.
     * a spatial dyadic mapping the motion space to the force space.
     *
     * \ingroup iDynTreeCore
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class SpatialInertiaRaw
    {
    private:
        double m_mass; /** mass */
        double m_mcom[3]; /** first moment of mass (i.e. mass * center of mass */
        RotationalInertiaRaw m_rotInertia; /** rotational inertia */

    public:
        SpatialInertiaRaw();

        /**
         * @param mass mass of the rigid body
         * @param com center of mass of the rigid body, expressed in the frame
         *            in which the spatial inertia is expressed
         * @param rotInertia rotational inertia expressed with respect to the origin of the frame.
         *
         * \warning the KDL::RigidBodyInertia class has a similar constructor, but in that one
         *          the rotational inerta in input is expressed in the center of mass of the body.
         */
        SpatialInertiaRaw(const double mass, const PositionRaw & com, const RotationalInertiaRaw & rotInertia);
        SpatialInertiaRaw(const SpatialInertiaRaw & other);
        virtual ~SpatialInertiaRaw();

        /** reset to zero (i.e. the inertia of body with zero pass) the SpatialInertia */
        void zero();
    };
}

#endif /* IDYNTREE_SPATIAL_FORCE_RAW_H */