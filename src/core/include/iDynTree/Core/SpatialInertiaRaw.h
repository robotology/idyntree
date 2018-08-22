/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SPATIAL_INERTIA_RAW_H
#define IDYNTREE_SPATIAL_INERTIA_RAW_H

#include <iDynTree/Core/RotationalInertiaRaw.h>

namespace iDynTree
{
    class PositionRaw;
    class SpatialForceVector;
    class SpatialMotionVector;

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
    protected:
        double m_mass; ///< Mass.
        double m_mcom[3]; ///< First moment of mass (i.e. mass * center of mass).
        RotationalInertiaRaw m_rotInertia; ///< Three dimensional rotational inertia.

    public:
        /**
         * Default constructor.
         * The data is not reset to zero for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        inline SpatialInertiaRaw() {}

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

        /**
         * Helper constructor-like function that takes mass, center of mass
         * and the rotational inertia expressed in the center of mass.
         *
         */
        void fromRotationalInertiaWrtCenterOfMass(const double mass, const PositionRaw & com, const RotationalInertiaRaw & rotInertia);


        /** multiplication operator
         *
         * overloading happens on proper classes
         *
         */


        /**
         * Getter functions
         *
         * \note for preserving consistency, no setters are implemented..
         *       if you want to modify a spatial inertia create a new one,
         *       and assign it to the spatial inertia that you want modify.
         *       Given that no memory allocation happens it should be still
         *       efficient.
         */
        double getMass() const;
        PositionRaw getCenterOfMass() const;
        const RotationalInertiaRaw& getRotationalInertiaWrtFrameOrigin() const;
        RotationalInertiaRaw getRotationalInertiaWrtCenterOfMass() const;


        /**
         * Function to combine the rigid body inertia of two different rigid bodies,
         * giving the rigid body inertia of of the rigid body obtanined by welding the two bodies.
         */
        static SpatialInertiaRaw combine(const SpatialInertiaRaw& op1, const SpatialInertiaRaw& op2);

        /**
         * Multiplication function
         *
         */
        SpatialForceVector multiply(const SpatialMotionVector & op) const;

        /** reset to zero (i.e. the inertia of body with zero mass) the SpatialInertia */
        void zero();
    };
}

#endif

