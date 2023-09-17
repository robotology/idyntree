// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SPATIAL_FORCE_VECTOR_H
#define IDYNTREE_SPATIAL_FORCE_VECTOR_H

#include <iDynTree/VectorFixSize.h>
#include <iDynTree/GeomVector3.h>
#include <iDynTree/SpatialVector.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates for any spatial force vector,
     * (i.e. vector form of an element of se*(3)).
     *
     * \ingroup iDynTreeCore
     *
     * A force spatial vector can be used to to described spatial momentum, wrench,
     * or their derivatives.
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * Transform::apply(SpatialForce),
     * for consistency with the iDynTree::Position class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class SpatialForceVector: public SpatialVector<SpatialForceVector>
    {
    public:
        /**
         * We use traits here to have the associations SpatialVector <=> Linear/Angular 3D vectors types
         * defined in a single place.
         */
        typedef SpatialMotionForceVectorT_traits<SpatialForceVector>::LinearVector3Type LinearVector3T;
        typedef SpatialMotionForceVectorT_traits<SpatialForceVector>::AngularVector3Type AngularVector3T;

        /**
         * Default constructor.
         * The data is not reset to zero for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline SpatialForceVector() {}
        SpatialForceVector(const LinearVector3T & _linearVec3, const AngularVector3T & _angularVec3);
        SpatialForceVector(const SpatialForceVector & other);
        SpatialForceVector(const SpatialVector<SpatialForceVector> & other);
        virtual ~SpatialForceVector();


        SpatialForceVector operator*(const double scalar) const;
    };
}

#endif /* IDYNTREE_SPATIAL_FORCE_VECTOR_H */
