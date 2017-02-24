/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINEAR_MOTION_VECTOR_3_H
#define IDYNTREE_LINEAR_MOTION_VECTOR_3_H

#include <iDynTree/Core/MotionVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    class Position;
    class PositionSemantics;
    class AngularMotionVector3;
    class AngularMotionVector3Semantics;

    /**
     * Class providing the semantics for any linear motion vector.
     */
    class LinearMotionVector3Semantics: public GeomVector3Semantics<LinearMotionVector3Semantics>
    {
    protected:
        int point;

    public:
        /**
         * Constructors:
         */
        inline LinearMotionVector3Semantics() {}
        LinearMotionVector3Semantics(int _point, int _body, int _refBody, int _coordinateFrame);
        LinearMotionVector3Semantics(const LinearMotionVector3Semantics & other);

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        bool changePoint(const PositionSemantics & newPoint,
                         const AngularMotionVector3Semantics & otherAngular,
                         LinearMotionVector3Semantics & resultLinear) const;

        static bool compose(const LinearMotionVector3Semantics & op1,
                            const LinearMotionVector3Semantics & op2,
                            LinearMotionVector3Semantics & result);
    };


    /**
     * Class providing the raw coordinates and semantics for any linear motion vector
     *
     * \ingroup iDynTreeCore
     *
     * A linear motion vector can be used to describe any linear velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class LinearMotionVector3: public MotionVector3<LinearMotionVector3>
    {
    public:
        typedef MotionForce_traits<LinearMotionVector3>::SemanticsType SemanticsType;

        /**
         * constructors
         */
        LinearMotionVector3();
        LinearMotionVector3(const double x, const double y, const double z);
        LinearMotionVector3(const double* in_data, const unsigned int in_size);
        LinearMotionVector3(const LinearMotionVector3 & other);
        LinearMotionVector3(const Vector3& other);

        /**
         * Geometric operations
         */
        const LinearMotionVector3 changePoint(const Position & newPoint,
                                              const AngularMotionVector3 & otherAngular) const;
    };

    typedef LinearMotionVector3 LinVelocity;
    typedef LinearMotionVector3 LinAcceleration;
}

#endif /* IDYNTREE_LINEAR_MOTION_VECTOR_3_H */
