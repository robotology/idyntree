/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 : 
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ANGULAR_FORCE_VECTOR_3_H
#define IDYNTREE_ANGULAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>
#include <iDynTree/Core/Utils.h>

namespace iDynTree
{
    class Position;
    class PositionSemantics;
    class LinearForceVector3;
    class LinearForceVector3Semantics;

    /**
     * Class providing the semantics for any angular force vector (torque or angular momentum).
     */
    class
    AngularForceVector3Semantics: public ForceVector3Semantics<AngularForceVector3Semantics>
    {
    protected:
        int point;

    public:
        /**
         * Constructors:
         */
        inline AngularForceVector3Semantics() {}
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        AngularForceVector3Semantics(int _point, int _body, int _refBody, int _coordinateFrame);
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        AngularForceVector3Semantics(const AngularForceVector3Semantics & other);

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        bool changePoint(const PositionSemantics & newPoint,
                         const LinearForceVector3Semantics & otherLinear,
                         AngularForceVector3Semantics & resultAngular) const;

        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        static bool compose(const AngularForceVector3Semantics & op1,
                            const AngularForceVector3Semantics & op2,
                            AngularForceVector3Semantics & result);
    };


    /**
     * Class providing the raw coordinates and semantics for any torque vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe an angular momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularForceVector3: public ForceVector3<AngularForceVector3>
    {
    public:
        typedef MotionForce_traits<AngularForceVector3>::SemanticsType SemanticsType;

        /**
         * constructors
         */
        inline AngularForceVector3() {}
        AngularForceVector3(const double x, const double y, const double z);
        AngularForceVector3(const double* in_data, const unsigned int in_size);
        AngularForceVector3(const AngularForceVector3 & other);
        AngularForceVector3(const Vector3& other);

        /**
         * Geometric operations
         */
        AngularForceVector3 changePoint(const Position & newPoint,
                                        const LinearForceVector3 & otherLinear) const;
    };

    typedef AngularForceVector3 AngMomentum;
    typedef AngularForceVector3 Torque;
}

#endif /* IDYNTREE_ANGULAR_FORCE_VECTOR_3_H */
