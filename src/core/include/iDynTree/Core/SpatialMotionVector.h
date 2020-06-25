/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SPATIAL_MOTION_RAW_H
#define IDYNTREE_SPATIAL_MOTION_RAW_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/SpatialVector.h>

namespace iDynTree
{
    class SpatialForceVector;
    class Transform;
    class Dummy {};

    /**
     * Class providing the raw coordinates for any motion spatial vector
     * (i.e. vector form of an element of se(3)).
     *
     * \ingroup iDynTreeCore
     *
     * A motion spatial vector can be used to to describe a twist, twist acceleration
     * or their derivatives.
     *
     * A generic motion spatial vector can also be used to store the logarithm of
     * an iDynTree::Transform (i.e. an element of SE(3)).
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * TransformRaw::apply(SpatialMotionRaw),
     * for consistency with the iDynTree::PositionRaw class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */

    class SpatialMotionVector: public SpatialVector<SpatialMotionVector>
    {
    public:
        /**
         * constructors
         */
        inline SpatialMotionVector() {}
        SpatialMotionVector(const LinearMotionVector3 & _linearVec3, const AngularMotionVector3 & _angularVec3);
        SpatialMotionVector(const SpatialMotionVector & other);
        SpatialMotionVector(const SpatialVector<SpatialMotionVector> & other);

        /**
         * Multiplication for a scalar.
         * Mainly used if SpatialMotionVector is used to represent a motion subspace.
         */
        SpatialMotionVector operator*(const double scalar) const;

/**
         * Cross product between two 6D motion vectors
         * \f$ V_1 = \begin{bmatrix} v_1 \\ \omega_1 \end{bmatrix} \f$
         * and
         * \f$ V_2 = \begin{bmatrix} v_2 \\ \omega_2 \end{bmatrix} \f$
         *
         * Returns:
         * \f[
         *   V_1 \times V_2 =
         *  \begin{bmatrix}
         *   v_1 \times \omega_2 + \omega_1 \times v_2 \\
         *   \omega_1 \times \omega_2
         *  \end{bmatrix}
         * \f]
         */
        SpatialMotionVector cross(const SpatialMotionVector& other) const;

        /**
         * Cross product between a 6D motion vector \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix} \f$ and
         * a 6D force vector \f$ F = \begin{bmatrix} f \\ \mu \end{bmatrix} \f$.
         *
         * Returns:
         * \f[
         *   V \bar{\times}^* F =
         *  \begin{bmatrix}
         *   \omega \times f \\
         *   v \times f + \omega \times \mu
         *  \end{bmatrix}
         * \f]
         */
        SpatialForceVector cross(const SpatialForceVector& other) const;

        /**
         * Cross product matrices
         */

        /**
         * If this object is \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix}  \f$,
         * return the 6x6 matrix \f$ V\times \f$
         * such that, if U is a SpatialMotionVector :
         * \f[
         *   (V \times) U = V\texttt{.cross}(U)
         * \f]
         *
         * The returned matrix is then the following one:
         * \f[
         *   V \times =
         *  \begin{bmatrix}
         *   \omega \times & v \times \\
         *    0_{3\times3}   & \omega \times
         *  \end{bmatrix}
         * \f]
         */
        Matrix6x6 asCrossProductMatrix() const;

        /**
         * If this object is \f$ V =  \begin{bmatrix} v \\ \omega \end{bmatrix} \f$, return the 6x6 matrix \f$ V\times \f$
         * such that, if F is a SpatialForceVector :
         * \f[
         *   (V \bar{\times}^*) F = V\texttt{.cross}(F)
         * \f]
         *
          The returned matrix is then the following one:
         * \f[
         *   V \bar{\times}^* =
         *  \begin{bmatrix}
         *   \omega \times &  0_{3\times3} \\
         *    v \times   & \omega \times
         *  \end{bmatrix}
         * \f]
         */
        Matrix6x6 asCrossProductMatrixWrench() const;


        /**
         * Exp mapping between a  generic element of se(3) (iDynTree::SpatialMotionVector)
         * to the corresponding element of SE(3) (iDynTree::Transform).
         */
        Transform exp() const;
    };

}

#endif
