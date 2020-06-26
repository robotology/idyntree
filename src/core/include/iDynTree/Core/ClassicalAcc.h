/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_CLASSICAL_ACC_H
#define IDYNTREE_CLASSICAL_ACC_H

#include <iDynTree/Core/VectorFixSize.h>

namespace iDynTree
{
    class RotationRaw;
    class SpatialAcc;
    class Twist;

    /**
     * Class representing a classical 6D acceleration, i.e. the concatenation
     * of the 3d vector of the acceleration of a point and of the 3d vector
     * of the derivative of an angular velocity.
     *
     * \note Given that the classical acceleration is just a combination of two
     *       3d vectors and not a spatial quantity, you
     *       cannot directly express a classical acceleration in
     *       a different reference frame.  You have to convert it before
     *       in a spatial acceleration, and then transform the spatial acceleration
     *       in a new reference frame (and the corresponding twist) and then
     *       convert them back to a classical acceleation.
     *       However you can still change the orientation frame of the two 3d vector
     *       that compose the classical acceleration without any conversion.
     *
     *
     *
     * \ingroup iDynTreeCore
     */
    class ClassicalAcc: public Vector6
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to the default for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline ClassicalAcc() {}
        ClassicalAcc(const double* in_data, const unsigned int in_size);
        ClassicalAcc(const ClassicalAcc& other);

        /* Geometric operations */
        const ClassicalAcc & changeCoordFrame(const RotationRaw & newCoordFrame);

        /** constructor helpers */
        static ClassicalAcc Zero();

        Vector3 getLinearVec3() const;
        Vector3 getAngularVec3() const;

        /**
         * Build a classical acceleration from a spatial acceleation and spatial twist
         * of a body.
         */
        void fromSpatial(const SpatialAcc& spatialAcc, const Twist& vel);

        /**
         * Convert a classical acceleation to a spatial one.
         */
        void toSpatial(SpatialAcc & spatialAcc, const Twist & vel) const;

    };
}

#endif
