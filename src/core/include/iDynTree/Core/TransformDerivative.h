/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TRANSFORM_DERIVATIVE_H
#define IDYNTREE_TRANSFORM_DERIVATIVE_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>

namespace iDynTree
{
    class Transform;
    class ArticulatedBodyInertia;
    class SpatialForceVector;

    /**
     * Class representing the derivative of Transform object
     *
     * \ingroup iDynTreeCore
     *
     * \note Even if most methods documentation are written
     *       assuming that this class represents the derivative
     *       of a Transform with respect to time, it can be used
     *       to represent a derivative with respect to an arbitrary
     *       variable.
     *
     */
    class TransformDerivative
    {
    protected:
        /**
         * \brief The derivative of the translation part of a Transform
         */
        Vector3 posDerivative;

        /**
         * \brief The derivative of the rotation part of a Transform
         */
        Matrix3x3 rotDerivative;

    public:
        /**
         * Default constructor.
         * The data is not reset to the default for perfomance reasons.
         * Please initialize the data in the class before any use.
         */
        TransformDerivative() {}

        /**
         * Constructor from a Matrix3x3 and a Vector3 .
         * The matrix represent the derivative of a rotation matrix,
         * while the vector the derivative of the position part of the tranform.
         */
        TransformDerivative(const Matrix3x3 & _rotDeriv, const Vector3 & posDeriv);

        /**
         * Copy constructor: create a TransformDerivative from another TransformDerivative.
         */
        TransformDerivative(const TransformDerivative & other);

        /**
         * Default destructor.
         */
        ~TransformDerivative() {}

        /**
         * Get the derivative of the rotation part of the transform
         *
         */
        const Matrix3x3 & getRotationDerivative() const;

        /**
         * Get the derivative of the  translation part of the transform
         *
         */
        const Vector3 & getPositionDerivative() const;

        /**
         * Set the derivative of the rotation part of the transform
         *
         */
        void setRotationDerivative(const Matrix3x3 & rotationDerivative);

        /**
         * \brief Set the derivative of translation part of the transform
         *
         */
        void setPositionDerivative(const Vector3 & positionDerivative);

        /**
         * Return a zero transfom derivative
         *
         * @return an zero transform derivative.
         */
        static TransformDerivative Zero();

        /**
         * @name Raw data accessors.
         *
         * For several applications it may be useful to access the fransform
         * contents in the raw form of homogeneous matrix or an adjoint matrix.
         */
        ///@{

        /**
         * Return the derivative of the 4x4 homogeneous matrix representing the transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}^{\texttt{refFrame}} \dot{H}_{\texttt{frame}} =
         * \begin{bmatrix}
         *    {}^{\texttt{refOrient}} \dot{R}_{\texttt{orient}} & {}^{\texttt{refOrient}} \dot{p}_{\texttt{refPoint},\texttt{point}} \\
         *    0_{1\times3} & 1
         * \end{bmatrix}
         * \f]
         *
         * For details on this notation, check the Transform::asHomogeneousTransform() method .
         *
         */
        Matrix4x4 asHomogeneousTransformDerivative() const;

        /**
         * Return the derivative of the  6x6 adjoint matrix representing this transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}^{\texttt{refFrame}} X_{\texttt{frame}} =
         * \begin{bmatrix}
         *    \dot{R} &
         *    \dot{p} \times R + p \times \dot{R} \\
         *    0_{3\times3} &
         *    \dot{R}
         * \end{bmatrix}
         * \f]
         *
         * Where \f$R = {}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3\times3}\f$
         * is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         *
         * \f$p = p_{\texttt{refPoint},\texttt{point}} \in \mathbb{R}^3\f$
         * is the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame \f$p \times\f$ is the skew simmetric
         * matrix such that \f$(p \times) v = p \times v\f$ .
         *
         * Given that the TransformDerivative object contains only \f$ \dot{R} \f$ and
         * \f$ \dot{p} \f$, you need to provide \f$ R \f$ and \f$ p \f$ to this method
         * by passing the relative Transform object.
         *
         * \warning Note that in iDynTree the matrix are stored
         *          in row major order, and the 6d quantites are
         *          serialized in linear/angular order.
         *
         */
        Matrix6x6 asAdjointTransformDerivative(const Transform & transform) const;

        /**
         * Return the 6x6 adjoint matrix (for wrench) representing this transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}_{\texttt{refFrame}} X^{\texttt{frame}} =
         * \begin{bmatrix}
         *    \dot{R} &
         *      0_{3\times3} \\
         *    \dot{p} \times R + p \times \dot{R} &
         *    \dot{R}
         * \end{bmatrix}
         * \f]
         *
         * Where \f$R = {}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3\times3}\f$
         * is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         *
         * \f$p = p_{\texttt{refPoint},\texttt{point}} \in \mathbb{R}^3\f$
         * is the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame \f$p \times\f$ is the skew simmetric
         * matrix such that \f$(p \times) v = p \times v\f$ .
         *
         * Given that the TransformDerivative object contains only \f$ \dot{R} \f$ and
         * \f$ \dot{p} \f$, you need to provide \f$ R \f$ and \f$ p \f$ to this method
         * by passing the relative Transform object.
         *
         * \warning Note that in iDynTree the matrix are stored
         *          in row major order, and the 6d quantites are
         *          serialized in linear/angular order.
         *
         */
        Matrix6x6 asAdjointTransformWrenchDerivative(const Transform & transform) const;

        /**
         * Multiply a transform derivative for a transform.
         *
         * This operation is useful to compose the derivative of a transform
         * for a constant transform.
         *
         * If this transform derivative represent the derivative of a transform
         * with respect to time \f$ ~^a \dot{H}_b \f$, and we have a constant
         * transform \f$ ~^b H_c \f$, then this operation returns the derivative
         * of the transform \f$ ~^a H_c \f$ :
         * \f[
         * ~^a \dot{H}_c = ~^a \dot{H}_b ~^b H_c
         * \f]
         *
         * \warning the otherTransform must be a quantity that does not depend
         *          on the derivation variable.
         */
        TransformDerivative operator*(const Transform & otherTransform) const;

        /**
         * Get the derivative of the inverse of the transform.
         *
         * If this TransformDerivative is \f$ ~^a \dot{H}_b \f$ and the
         * transform argument is \f$ ~^a H_b  \f$, returns the derivative of
         * the inverse transform \f$ ~^b {H}_a  \f$ computed as:
         * \f[
         * ~^b \dot{H}_a = - ~^b {H}_a ~^a \dot{H}_b ~^b H_a
         * \f]
         *
         */
        TransformDerivative derivativeOfInverse(const Transform & transform) const;

        /**
         * Given a articulated inertia \f$I\f$ (other argument), this TransformDerivative \f$ ~^a\dot{X}_b \f$
         * and a transform \f$ ~^a{X}_b  \f$ return the articulated body inertia:
         * \f[
         *   ~_a \dot{X}^b Ia ~^a X_b +  ~_a {X}^b Ia ~^a \dot{X}_b
         * \f]
         */
        ArticulatedBodyInertia transform(const Transform & transform, ArticulatedBodyInertia& other);

        /**
         * Given a SpatialForceVector \f$F\f$ (other argument), this TransformDerivative \f$ ~^a\dot{X}_b  \f$
         * and a transform \f$ ~^a{X}_b \f$ return the spatial force vector:
         * \f[
         *   ~_a \dot{X}^b F
         * \f]
         */
        SpatialForceVector transform(const Transform & transform, SpatialForceVector& other);

        /**
         * Given a SpatialMotionVector \f$V\f$ (other argument), this TransformDerivative \f$ ~^a\dot{X}_b  \f$
         * and a transform \f$ ~^a{X}_b \f$ return the spatial force vector:
         * \f[
         *   ~^a \dot{X}_b F
         * \f]
         */
        SpatialMotionVector transform(const Transform & transform, SpatialMotionVector& other);
    };
}

#endif
