/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
        LinearForceVector3Semantics();
        LinearForceVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        LinearForceVector3Semantics(const LinearForceVector3Semantics & other);
        ~LinearForceVector3Semantics();
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
        LinearForceVector3();
        LinearForceVector3(const double x, const double y, const double z);
        LinearForceVector3(const double* in_data, const unsigned int in_size);
        LinearForceVector3(const LinearForceVector3 & other);
        virtual ~LinearForceVector3();
    };

    typedef LinearForceVector3 LinMomentum;
    typedef LinearForceVector3 Force;
}

#endif /* IDYNTREE_LINEAR_FORCE_VECTOR_3_H */
