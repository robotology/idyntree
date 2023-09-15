// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_CLASSICAL_ACC_H
#define IDYNTREE_CLASSICAL_ACC_H

#include <iDynTree/VectorFixSize.h>

namespace iDynTree
{
    class Rotation;
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
        const ClassicalAcc & changeCoordFrame(const Rotation & newCoordFrame);

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
