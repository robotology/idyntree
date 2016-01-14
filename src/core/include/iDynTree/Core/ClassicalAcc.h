/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_CLASSICAL_ACC_H
#define IDYNTREE_CLASSICAL_ACC_H

#include <iDynTree/Core/VectorFixSize.h>

namespace iDynTree
{
    class RotationRaw;
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
     * Currently this class does not support semantics.
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
        inline ClassicalAcc() {};
        ClassicalAcc(const double* in_data, const unsigned int in_size);
        ClassicalAcc(const ClassicalAcc& other);

        /* Geometric operations */
        const ClassicalAcc & changeCoordFrame(const RotationRaw & newCoordFrame);

        /** constructor helpers */
        static ClassicalAcc Zero();

    };
}

#endif