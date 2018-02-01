/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ANGULAR_MOTION_VECTOR_3_H
#define IDYNTREE_ANGULAR_MOTION_VECTOR_3_H

#include <iDynTree/Core/MotionVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the semantics for any angular motion vector (angular velocity or acceleration).
     */
    class AngularMotionVector3Semantics: public GeomVector3Semantics<AngularMotionVector3Semantics>
    {
    public:
        /**
         * Constructors:
         */
        AngularMotionVector3Semantics();
        AngularMotionVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        AngularMotionVector3Semantics(const AngularMotionVector3Semantics & other);
    };


    /**
     * Class providing the raw coordinates and semantics for any angular motion vector
     *
     * \ingroup iDynTreeCore
     *
     * An angular motion vector can be used to describe any angular velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     * The angular motion vector is also used to represent a generic element
     * of the so(3) spaces.
     *
     */
    class AngularMotionVector3: public MotionVector3<AngularMotionVector3>
    {
    public:
        typedef MotionForce_traits<AngularMotionVector3>::SemanticsType SemanticsType;

        /**
         * constructors
         */
        inline AngularMotionVector3() {}
        AngularMotionVector3(double x, double y, double z);
        AngularMotionVector3(const double* in_data, const unsigned int in_size);
        AngularMotionVector3(const AngularMotionVector3 & other);
        AngularMotionVector3(const Vector3 & other);

        /**
         * Exp mapping between a  generic element of so(3) (iDynTree::AngularMotionVector3)
         * to the corresponding element of SO(3) (iDynTree::Rotation).
         */
        Rotation exp() const;
    };

    typedef AngularMotionVector3 AngVelocity;
    typedef AngularMotionVector3 AngAcceleration;
}

#endif /* IDYNTREE_ANGULAR_MOTION_VECTOR_3_H */
