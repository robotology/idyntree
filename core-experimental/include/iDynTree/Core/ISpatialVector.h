/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_SPATIAL_VECTOR_H
#define IDYNTREE_I_SPATIAL_VECTOR_H


namespace iDynTree
{
    /**
     * Class providing an interface to any spatial motion or force vector, which provides
     * raw coordinates.
     *
     * \ingroup iDynTreeCore
     *
     * A motion spatial vector can be used to describe twist, spatial acceleration,
     * and derivatives.
     *
     * A force spatial vector can be used to describe spatial momentum, wrench,
     * and derivatives.
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * TransformRaw::apply(SpatialForceRaw),
     * for consistency with the iDynTree::PositionRaw class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class ISpatialVector
    {
    public:
        virtual ~ISpatialVector() {}
    };
}

#endif /* IDYNTREE_I_SPATIAL_VECTOR_H */