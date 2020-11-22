/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TRANSFORM_H
#define IDYNTREE_TRANSFORM_H

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/MatrixFixSize.h>

#include <string>

namespace iDynTree
{
    class Position;
    class Rotation;
    class Wrench;
    class Twist;
    class SpatialMomentum;
    class SpatialAcc;
    class SpatialMotionVector;
    class SpatialForceVector;
    class SpatialInertia;
    class ArticulatedBodyInertia;

    /**
     * Class representation the relative displacement between two different frames.
     *
     * \ingroup iDynTreeCore
     *
     * \image html transform.svg
     *
     *
     * This class is designed to be an easy to use proxy to perform change of frame of
     * expression for iDynTree::Position, iDynTree::Twist, iDynTree::Wrench and other data
     * structure in \ref iDynTreeCore.  For this reason the class is called "Transform", because it will be mainly
     * used to transform quantities between frames.
     *
     * Given that this class it may used to represent homogeneous transform matrix as well as adjoint
     * matrix, no raw access to the underline storage ( data() method ) is provided, because it does not
     * have a canonical representation. Instead, access is given to the two elements of a transform:
     *  The position vector \f${}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}}\f$ and
     * the rotation matrix \f${}^{\texttt{refOrient}} R_{\texttt{orient}}\f$.
     *
     * We will indicate this tranform as \f$( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )\f$
     *
     * The frame whose origin is the reference point and whose orientation the reference orientation is often
     * indicated as \f$ \texttt{refFrame} = (\texttt{refPoint},\texttt{refOrient}) \f$.
     *
     * Similarly the frame whose origin is the point and whose orientation is the orientation is indicated
     * as \f$ \texttt{frame} = (\texttt{point},\texttt{orient}) \f$.
     *
     * This transform object can be obtained as the \f${}^{\texttt{refFrame}} H_{\texttt{frame}}\f$  4x4 homogeneous matrix
     * using the asHomogeneousTransform method, or as the \f${}^{\texttt{refFrame}} X_{\texttt{frame}}\f$ 6x6 adjoint matrix using the
     * asAdjointTransform .
     *
     *
     *
     */
    class Transform
    {
    protected:
        /**
         * \brief  The position part of the transform.
         *
         * The 3d vector \f${}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}}\f$,
         * that is  the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame.
         */
        Position pos;

        /**
         * \brief The rotation part of the transform
         *
         * Set the rotation matrix \f${}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3 \times 3}\f$,
         * that is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         */
        Rotation rot;

    public:
        /**
         * Default constructor.
         * The data is not reset to the default for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        Transform();

        /**
         * Constructor from a rotation and a point
         */
        Transform(const Rotation & _rot, const Position & origin);

        /**
        * Constructor from a Matrix4x4 object. It is equivalent of calling fromHomogeneousTransform()
        * @param transform The input homogeneous matrix
        */
        Transform(const Matrix4x4 & transform);

        /**
         * Copy constructor: create a Transform from another Transform.
         */
        Transform(const Transform & other);

        /**
         * Set rotation and translation from a iDynTree::Matrix4x4 object
         * @param transform The input homogeneous matrix
         */
        void fromHomogeneousTransform(const Matrix4x4& transform);

        /**
         * Assigment operator
         *
         * @param other the rhs
         *
         * @return *this equal to the other object
         */
        Transform& operator=(const Transform& other);

        /**
         * Get the rotation part of the transform
         *
         * Get the rotation matrix \f${}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3 \times 3}\f$,
         * that is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         */
        const Rotation & getRotation() const;

        /**
         * \brief Get the translation part of the transform
         *
         * Get 3d vector \f${}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}}\f$,
         * that is  the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame.
         */
        const Position & getPosition() const;

        /**
         * Set the rotation part of the transform
         *
         * Set the rotation matrix \f${}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3 \times 3}\f$,
         * that is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         */
        void setRotation(const Rotation & rotation);

        /**
         * \brief Set the translation part of the transform
         *
         * Set 3d vector \f${}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}}\f$,
         * that is  the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame.
         */
        void setPosition(const Position & position);

        // geometric operations on 3x1 vectors (positions and rotations and homogemeous tranform)
        static Transform compose(const Transform & op1, const Transform & op2);
        static Transform inverse2(const Transform & trans);

        /**
         * \name Overloaded operators.
         *
         * This operators are used to change the frame in which a quantity is
         * expressed from the \f$\texttt{frame}\f$ to the \f$\texttt{refFrame}\f$ of
         * the transform.
         */
        ///@{

        /**
         * \brief Combine two transforms.
         *
         * If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         * and the argument is
         * \f[
         * (p',R') = ( {}^{\texttt{orient}} p_{\texttt{point},\texttt{newPoint}} , {}^{\texttt{orient}} R_{\texttt{newOrient}} )
         * \f].
         *
         * The resulting transform will be:
         * \f[
         * (p+Rp', RR') = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{newPoint}} , {}^{\texttt{refOrient}} R_{\texttt{newOrient}} )
         * \f].
         *
         * Notice that this is equivalent to multiply the associated homogemeous matrices:
         * \f[
         * {}^{\texttt{refFrame}} H_{\texttt{newFrame}} = {}^{\texttt{refFrame}} H_{\texttt{frame}} {}^{\texttt{frame}} H_{\texttt{newFrame}}
         * \f]
         * or the associated adjoint matrices :
         * \f[
         * {}^{\texttt{refFrame}} X_{\texttt{newFrame}} = {}^{\texttt{refFrame}} X_{\texttt{frame}} {}^{\texttt{frame}} X_{\texttt{newFrame}}
         * \f]
         */
        Transform operator*(const Transform & other) const;

        /**
         * Return the inverse of this Transform.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         * this function will return:
         * \f[
         * (-R^\top p , R^\top) = ( {}^{\texttt{orient}} p_{\texttt{point},\texttt{refPoint}} , {}^{\texttt{orient}} R_{\texttt{refOrient}} )
         * \f]
         *
         */
        Transform inverse() const;

        /**
         * Change reference frame of a point.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         *
         *  And the Position argument represent a point:
         * \f[
         *  p' = {}^{\texttt{orient}} p_{\texttt{point},\texttt{newPoint}}
         * \f]
         *
         * The result of the operation is:
         * \f[
         *  Rp' + p =  {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{newPoint}}
         * \f]
         *
         */
        Position operator*(const Position & other) const;

        /**
         * Change frame in which a Wrench is expressed.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         *
         * And the argument is a wrench:
         * \f[
         * {}_{\texttt{frame}} F =
         * \begin{bmatrix}
         * f \\ \tau
         * \end{bmatrix}
         * \f]
         *
         * The result of this operation is :
         * \f[
         * {}_{\texttt{refFrame}} F
         * =
         * {}_{\texttt{refFrame}}X^{\texttt{frame}}
         * {}_{\texttt{frame}} F
         * =
         * \begin{bmatrix}
         *    R &
         *    0_{3\times3} \\
         *    p \times R &
         *    R
         * \end{bmatrix}
         *  \begin{bmatrix}
         * f \\ \tau
         * \end{bmatrix}
         * =
          \begin{bmatrix}
         * Rf \\ p \times R f + R\tau
         * \end{bmatrix}
         * \f]
         */
        Wrench   operator*(const Wrench & other) const;

        /**
         * Change the frame in which a Twist is expressed.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         *
         * And the argument is a twist:
         * \f[
         * {}^{\texttt{frame}} V =
         * \begin{bmatrix}
         * v \\ \omega
         * \end{bmatrix}
         * \f]
         *
         * The result of this operation is :
         * \f[
         * {}^{\texttt{refFrame}} V
         * =
         * {}^{\texttt{refFrame}}X_{\texttt{frame}}
         * {}^{\texttt{frame}} V
         * =
         * \begin{bmatrix}
         *    R &
         *    p \times R\\
         *     0_{3\times3}  &
         *    R
         * \end{bmatrix}
         *  \begin{bmatrix}
         * v \\ \omega
         * \end{bmatrix}
         * =
          \begin{bmatrix}
         * R v + p \times R \omega \\ R\omega
         * \end{bmatrix}
         * \f]
         */
        Twist    operator*(const Twist  & other) const;

        /**
         * Change the frame in which a SpatialMomentum is expressed.
         *
         * Check the operator*(const Wrench & other) documentation
         * for the mathematical details.
         */
        SpatialMomentum operator*(const SpatialMomentum & other) const;

        /**
         * Change the frame in which a SpatialAcc is expressed.
         *
         * Check the operator*(const Twist & other) documentation
         * for the mathematical details.
         */
        SpatialAcc   operator*(const SpatialAcc & other) const;

        /**
         * Change the frame in which a SpatialMotionVector is expressed.
         *
         * Check the operator*(const Twist & other) documentation
         * for the mathematical details.
         */
        SpatialMotionVector   operator*(const SpatialMotionVector & other) const;

        /**
         * Change the frame in which a SpatialForceVector is expressed.
         *
         * Check the operator*(const Wrench & other) documentation
         * for the mathematical details.
         */
        SpatialForceVector   operator*(const SpatialForceVector & other) const;

        /**
         * Change the frame in which a SpatialInertia is expressed.
         *
         */
        SpatialInertia operator*(const SpatialInertia  & other) const;

        /**
         * Change the frame in which a ArticulatedBodyInertia is expressed.
         *
         */
        ArticulatedBodyInertia operator*(const ArticulatedBodyInertia  & other) const;


        /**
         * Change the frame in which a Direction is expressed.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         *
         * And the argument is a direction represented by a unit norm 3d vector :
         * \f[
         *   {}^{\texttt{orient}} d
         * \f]
         *
         * The result of this operation is:
         * \f[
         *   {}^{\texttt{refOrient}} d = R  {}^{\texttt{orient}} d
         * \f]
         *
         */
        Direction      operator*(const Direction & other) const;

        /**
         * Change the frame in which a Axis is expressed.
         *
         *  If this object is
         * \f[
         * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
         * \f]
         *
         * And the argument is an axis, specified by the axis origin and the axis direction:
         * \f[
         *  {}^{\texttt{frame}} A = ({}^{\texttt{orient}} p_{\texttt{point},\texttt{axisOrigin}}  , {}^{\texttt{orient}} d) = (p',d)
         * \f]
         *
         * The result of this operation is:
         * \f[
         *  {}^{\texttt{refFrame}} A =  ({}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{axisOrigin}}  , {}^{\texttt{refOrient}} d)  =  ( Rp' + p , Rd)
         * \f]
         *
         */
        Axis           operator*(const Axis & other) const;
        ///@}

        /**
         * Return an identity transfom
         *
         * Set the rotation to the identity and the translation to 0 :
         * \f[
         * (0_{3 \times 1}, I_{3 \times 3})
         * \f]
         *
         * @return an identity transform.
         */
        static Transform Identity();

        /**
         * @name Raw data accessors.
         *
         * For several applications it may be useful to access the fransform
         * contents in the raw form of homogeneous matrix or an adjoint matrix.
         */
        ///@{

        /**
         * Return the 4x4 homogeneous matrix representing the transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}^{\texttt{refFrame}} H_{\texttt{frame}} =
         * \begin{bmatrix}
         *    {}^{\texttt{refOrient}} R_{\texttt{orient}} & {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} \\
         *    0_{1\times3} & 1
         * \end{bmatrix}
         * \f]
         * Where \f${}^{\texttt{refOrient}} R_{\texttt{orient}} \in \mathbb{R}^{3 \times 3}\f$ is the rotation matrix that takes
         * a 3d vector expressed in the orientationFrame and transform it
         * in a 3d vector expressed in the reference orientation frame.
         *
         * \f${}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} \in \mathbb{R}^3\f$
         * is the vector between the point and the
         * reference point, pointing toward the point and expressed
         * in the reference orientation frame.
         *
         */
        Matrix4x4 asHomogeneousTransform() const;

        /**
         * Return the 6x6 adjoint matrix representing this transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}^{\texttt{refFrame}} X_{\texttt{frame}} =
         * \begin{bmatrix}
         *    R &
         *    p \times R \\
         *    0_{3\times3} &
         *    R
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
         * \warning Note that in iDynTree the matrix are stored
         *          in row major order, and the 6d quantites are
         *          serialized in linear/angular order.
         *
         */
        Matrix6x6 asAdjointTransform() const;

        /**
         * Return the 6x6 adjoint matrix (for wrench) representing this transform.
         *
         * The returned matrix is the one with this structure:
         *
         * \f[
         *  {}_{\texttt{refFrame}} X^{\texttt{frame}} =
         * \begin{bmatrix}
         *    R &
         *      0_{3\times3} \\
         *    p \times R &
         *    R
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
         * \warning Note that in iDynTree the matrix are stored
         *          in row major order, and the 6d quantites are
         *          serialized in linear/angular order.
         *
         */
        Matrix6x6 asAdjointTransformWrench() const;

        /*
         * Exp mapping between a  generic element of se(3) (iDynTree::SpatialMotionVector)
         * to the corresponding element of SE(3) (iDynTree::Transform).
         */
        SpatialMotionVector log() const;

        ///@}

        /**
         * @name Output helpers.
         *  Function to print out the Transform.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };
}

#endif
