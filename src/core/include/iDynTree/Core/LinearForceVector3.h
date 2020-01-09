/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_LINEAR_FORCE_VECTOR_3_H
#define IDYNTREE_LINEAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the semantics for any angular motion vector (angular velocity or acceleration).
     */
    class LinearForceVector3Semantics: public ForceVector3Semantics<LinearForceVector3Semantics>
    {
    public:
        /**
         * Constructors:
         */
        inline LinearForceVector3Semantics() {}
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        LinearForceVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        LinearForceVector3Semantics(const LinearForceVector3Semantics & other);
    };


    /**
     * Class providing the raw coordinates and semantics for any linear force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe any linear momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class LinearForceVector3: public ForceVector3<LinearForceVector3>
    {
    public:
        typedef MotionForce_traits<LinearForceVector3>::SemanticsType SemanticsType;

        /**
         * constructors
         */
        inline LinearForceVector3() {}
        LinearForceVector3(const double x, const double y, const double z);
        LinearForceVector3(const double* in_data, const unsigned int in_size);
        LinearForceVector3(const LinearForceVector3 & other);
        LinearForceVector3(const Vector3 & other);
    };

    typedef LinearForceVector3 LinMomentum;
    typedef LinearForceVector3 Force;
}

#endif /* IDYNTREE_LINEAR_FORCE_VECTOR_3_H */
