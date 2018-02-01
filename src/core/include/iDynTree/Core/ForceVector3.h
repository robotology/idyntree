/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_FORCE_VECTOR_3_H
#define IDYNTREE_FORCE_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

#define FORCEVECTOR3SEMANTICS_TEMPLATE_HDR \
template <class ForceTSemantics>

#define FORCEVECTOR3_TEMPLATE_HDR \
template <class ForceT>

namespace iDynTree
{
    /**
     * Template class providing the semantics for any Force relation vector.
     */
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    class ForceVector3Semantics: public GeomVector3Semantics<ForceTSemantics>
    {
    public:
        /**
         * Constructors:
         */
        inline ForceVector3Semantics() {}
        ForceVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        ForceVector3Semantics(const ForceVector3Semantics & other);

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        static bool compose(const ForceVector3Semantics & op1,
                            const ForceVector3Semantics & op2,
                            ForceVector3Semantics & result);

        static bool inverse(const ForceVector3Semantics & op,
                            ForceVector3Semantics & result);
    };


    /**
     * Class providing the raw coordinates and semantics for any force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe a linear or angular momentum or force.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every force vectors.
     *
     */
    FORCEVECTOR3_TEMPLATE_HDR
    class ForceVector3: public GeomVector3<ForceT>
    {
    public:
        /**
         * constructors
         */
        inline ForceVector3() {}
        ForceVector3(const double* in_data, const unsigned int in_size);
        ForceVector3(const ForceVector3 & other);
    };

}

#endif /* IDYNTREE_FORCE_VECTOR_3_H */
