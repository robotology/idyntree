// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_ROTATION_H
#define IDYNTREE_ROTATION_H

#include <string>
#include <iDynTree/GeomVector3.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/Utils.h>

namespace iDynTree
{
    class Position;
    class Twist;
    class SpatialAcc;
    class Wrench;
    class Direction;
    class Axis;
    class SpatialAcc;
    class SpatialMomentum;
    class ClassicalAcc;
    class RotationalInertia;
    class SpatialMotionVector;
    class SpatialForceVector;
    class ArticulatedBodyInertia;

    /**
     * Class representation the rotation of an orientation frame
     * with respect to a reference orientation frame, expressed as a Rotation matrix.
     *
     * \ingroup iDynTreeCore
     *
     * The semantics for this class is based on the OrientationCoord in:
     *
     * De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
     * (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
     * IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
     * URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
     *
     * Storage for the Orientation:
     *
     * The rotation matrix representation of the orientation, stored in row major order,
     * inside a Matrix3x3 parent object.
     *
     * \warning This class uses for convenience the Matrix3x3 as a public parent.
     *          Notice that using this methods you can damage the underlyng rotation matrix.
     *          In doubt, don't use them and rely on more high level functions.
     */
    class Rotation : public Matrix3x3
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to the identity matrix for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        Rotation();

        /**
         * Constructor from 9 doubles: initialize elements of the rotation matrix.
         */
        Rotation(double xx, double xy, double xz,
                 double yx, double yy, double yz,
                 double zx, double zy, double zz);

        /**
         * Copy constructor: create a Rotation from another Rotation.
         */
        Rotation(const Rotation & other);

        /**
         * Create a Rotation from a MatrixView.
         */
        Rotation(iDynTree::MatrixView<const double> other);

        /**
         * Constructor from a buffer of 9 doubles,
         * stored as a C-style array (i.e. row major).
         *
         */
        Rotation(const double* in_data,
                 const unsigned int in_rows,
                 const unsigned int in_cols);
        /**
         * Geometric operations.
         * For the inverse2() operation, both the forward and the inverse geometric relations have to
         * be expressed in the reference orientation frame!!
         *
         */
        const Rotation & changeOrientFrame(const Rotation & newOrientFrame);
        const Rotation & changeRefOrientFrame(const Rotation & newRefOrientFrame);
        const Rotation & changeCoordinateFrame(const Rotation& newCoordinateFrame);
        static Rotation compose(const Rotation & op1, const Rotation & op2);
        static Rotation inverse2(const Rotation & orient);
        Position changeCoordFrameOf(const Position & other) const;
        SpatialMotionVector changeCoordFrameOf(const SpatialMotionVector & other) const;
        SpatialForceVector  changeCoordFrameOf(const SpatialForceVector  & other) const;
        Twist changeCoordFrameOf(const Twist & other) const;
        SpatialAcc changeCoordFrameOf(const SpatialAcc & other) const;
        SpatialMomentum changeCoordFrameOf(const SpatialMomentum & other) const;
        Wrench changeCoordFrameOf(const Wrench & other) const;
        Direction changeCoordFrameOf(const Direction & other) const;
        Axis      changeCoordFrameOf(const Axis & other) const;
        ClassicalAcc changeCoordFrameOf(const ClassicalAcc & other) const;
        RotationalInertia changeCoordFrameOf(const RotationalInertia & other) const;


        /**
          * overloaded operators
          */
        Rotation& operator=(const Rotation & other);
        Rotation operator*(const Rotation & other) const;
        Rotation inverse() const;
        Position operator*(const Position & other) const;
        SpatialForceVector operator*(const SpatialForceVector & other) const;
        Twist    operator*(const Twist    & other) const;
        Wrench   operator*(const Wrench   & other) const;
        Direction operator*(const Direction & other) const;
        Axis      operator*(const Axis    & other) const;
        SpatialAcc      operator*(const SpatialAcc    & other) const;
        SpatialMomentum operator*(const SpatialMomentum   & other) const;
        ClassicalAcc    operator*(const ClassicalAcc    & other) const;
        RotationalInertia    operator*(const RotationalInertia    & other) const;

        /**
         * Log mapping between a  generic element of SO(3) (iDynTree::Rotation)
         * to the corresponding element of so(3) (iDynTree::AngularMotionVector).
         */
        AngularMotionVector3 log() const;

        /**
         * Set the rotation matrix as the passed rotation expressed in quaternion
         *
         * @note the quaternion is expressed as (real, imaginary) part with
         * real \f$\in \mathbb{R}\f$ and imaginary \f$\in \mathbb{R}^3\f$
         * @note the quaternion is normalized
         * @param quaternion the rotation expressed in quaternion
         */
        void fromQuaternion(const iDynTree::Vector4& quaternion);


        /**
         * @name Conversion to others represention of matrices.
         *
         */
        ///@{

         /**
         * Get a roll, pitch and yaw corresponding to this rotation.
         *
         * Get \f$ (r,p,y) \in ( (-\pi, \pi] \times (-\frac{\pi}{2}, \frac{\pi}{2}) \times (-\pi, \pi] ) \cup ( \{0\} \times \{-\frac{\pi}{2}\} \times (-\pi,\pi] ) \cup ( \{0\} \times \{\frac{\pi}{2}\} \times [-\pi,\pi) )\f$
         * such that
         * *this == RotZ(y)*RotY(p)*RotX(r)
         *
         * @param[out] r roll rotation angle
         * @param[out] p pitch rotation angle
         * @param[out] y yaw rotation angle
         */
        void getRPY(double & r, double & p, double &y) const;

        /**
         * Get a roll, pitch and yaw corresponding to this rotation,
         * as for getRPY, but return a vector with the output
         * parameters. This function is more suitable for bindings.
         *
         * @return the output vector with the r, p and y parameters.
         */
        iDynTree::Vector3 asRPY() const;

        /**
         * Get a unit quaternion corresponding to this rotation
         *
         * The quaternion is defined as [s, r]
         * where \f$s \in \mathbb{R}\f$ is the real and
         * \f$r \in \mathbb{R}^3\f$ is the imaginary part.
         *
         * The returned quaternion is such that *this is
         * equal to RotationFromQuaternion(quaternion).
         *
         * \note For each rotation, there are two quaternion
         * corresponding to it. In this method we return
         * the one that has the first non-zero (with a tolerance of 1e-7)
         * component positive. If the real part is non-zero, this
         * mean that we return the quaternion with positive real part.
         *
         * @param[out] quaternion the output quaternion
         */
        bool getQuaternion(iDynTree::Vector4& quaternion) const;

        /**
         * Get a unit quaternion corresponding to this rotation
         *
         * The unit quaternion is defined as [s, r]
         * where \f$s \in \mathbb{R}\f$ is the real and
         * \f$r \in \mathbb{R}^3\f$ is the imaginary part.
         *
         * The returned quaternion is such that *this is
         * equal to RotationFromQuaternion(quaternion).
         *
         * \note For each rotation, there are two quaternion
         * corresponding to it. In this method we return
         * the one that has the first non-zero (with a tolerance of 1e-7)
         * component positive. If the real part is non-zero, this
         * mean that we return the quaternion with positive real part.
         *
         * @param[out] s the real part
         * @param[out] r1 the first component of the imaginary part (i.e. i base)
         * @param[out] r2 the second component of the imaginary part (i.e. j base)
         * @param[out] r3 the third component of the imaginary part (i.e. k base)
         */
        bool getQuaternion(double &s, double &r1, double &r2, double &r3) const;

        /**
         * Get a unit quaternion corresponding to this rotation
         *
         * The quaternion is defined as [s, r]
         * where \f$s \in \mathbb{R}\f$ is the costituent and
         * \f$r \in \mathbb{R}^3\f$ is the imaginary part.
         *
         * The returned quaternion is such that *this is
         * equal to RotationFromQuaternion(quaternion).
         *
         * \note For each rotation, there are two quaternion
         * corresponding to it. In this method we return
         * the one that has the first non-zero (with a tolerance of 1e-7)
         * component positive. If the real part is non-zero, this
         * mean that we return the quaternion with positive real part.
         *
         * @return the output quaternion
         */
        iDynTree::Vector4 asQuaternion() const;

        ///@}

        /**
         * @name Initialization helpers.
         *
         */
        ///@{

        /**
         * Return a Rotation around axis X of given angle
         *
         * If \f$ \theta \f$ is the input angle, this function
         * returns the \f$  R_x(\theta) \f$ rotation matrix such that :
         * \f[
         *   R_x(\theta) =
         *  \begin{bmatrix}
         *      1 & 0             & 0              \\
         *      0 & \cos(\theta)  & - \sin(\theta) \\
         *      0 & \sin(\theta)  & \cos(\theta)   \\
         *  \end{bmatrix}
         * \f]
         *
         * @param angle the angle (in Radians) of the rotation arount the X axis
         */
        static Rotation RotX(const double angle);

        /**
         * Return a Rotation around axis Y of given angle
         *
         * If \f$ \theta \f$ is the input angle, this function
         * returns the \f$  R_y(\theta) \f$ rotation matrix such that :
         * \f[
         *   R_y(\theta) =
         *  \begin{bmatrix}
         *      \cos(\theta)      & 0             & \sin(\theta)   \\
         *      0                 & 1             & 0              \\
         *      -\sin(\theta)     & 0             & \cos(\theta)   \\
         *  \end{bmatrix}
         * \f]
         *
         *
         * @param angle the angle (in Radians) of the rotation arount the Y axis
         */
        static Rotation RotY(const double angle);

        /**
         * Return a Rotation around axis Z of given angle
         *
         * If \f$ \theta \f$ is the input angle, this function
         * returns the \f$  R_z(\theta) \f$ rotation matrix such that :
         * \f[
         *   R_z(\theta) =
         *  \begin{bmatrix}
         *      \cos(\theta)      & -\sin(\theta) & 0              \\
         *      \sin(\theta)      & \cos(\theta)  & 0              \\
         *      0                 & 0             & 1              \\
         *  \end{bmatrix}
         * \f]
         *
         *
         * @param angle the angle (in Radians) of the rotation arount the Z axis
         */
        static Rotation RotZ(const double angle);

        /**
         * Return a Rotation around axis given by direction of given angle
         *
         * If we indicate with \f$ d \in \mathbb{R}^3 \f$ the unit norm
         * of the direction, and with \f$ \theta \f$ the input angle, the return rotation
         * matrix \f$ R \f$ can be computed using the Rodrigues' rotation formula [1] :
         * \f[
         *  R = I_{3\times3} + d^{\wedge} \sin(\theta) + {d^{\wedge}}^2 (1-\cos(\theta))
         * \f]
         *
         * [1] : http://mathworld.wolfram.com/RodriguesRotationFormula.html
         * @param direction the Direction around with to rotate
         * @param angle the angle (in Radians) of the rotation arount the given axis
         */
        static Rotation RotAxis(const Direction & direction, const double angle);

        /**
         * Return the derivative of the RotAxis function with respect to the angle argument.
         *
         * If we indicate with \f$ d \in \mathbb{R}^3 \f$ the unit norm
         * of the direction, and with \f$ \theta \f$ the input angle, the derivative of the rotation
         * matrix \f$ \frac{\partial R}{\partial \theta} \f$ can be computed using the
         * derivative of the Rodrigues' rotation formula [1] :
         * \f[
         *  \frac{\partial R}{\partial \theta} = d^{\vee} \cos(\theta) + {d^{\vee}}^2 \sin(\theta)
         * \f]
         *
         * [1] : http://mathworld.wolfram.com/RodriguesRotationFormula.html
         *
         * @param direction the Direction around with to rotate
         * @param angle the angle (in Radians) of the rotation arount the given  axis
         */
        static Matrix3x3 RotAxisDerivative(const Direction & direction, const double angle);

        /**
         * Return a rotation object given Roll, Pitch and Yaw values.
         *
         * @note This is equivalent to RotZ(y)*RotY(p)*RotX(r) .
         * @note This method is compatible with the KDL::Rotation::RPY method.
         */
        static Rotation RPY(const double roll, const double pitch, const double yaw);

        /**
         * Return the right-trivialized derivative of the RPY function.
         *
         * If we indicate with \f$ rpy \in \mathbb{R}^3 \f$ the roll pitch yaw vector,
         * and with \f$  RPY(rpy) : \mathbb{R}^3 \mapsto SO(3) \f$ the function implemented
         * in the Rotation::RPY method, this method returns the right-trivialized partial
         * derivative of Rotation::RPY, i.e. :
         * \f[
         *    (RPY(rpy) \frac{\partial RPY(rpy)}{\partial rpy})^\vee
         * \f]
         */
        static Matrix3x3 RPYRightTrivializedDerivative(const double roll, const double pitch, const double yaw);

        /**
         * Return the rate of change of the right-trivialized derivative of the RPY function.
         *
         * If we indicate with \f$ rpy \in \mathbb{R}^3 \f$ the roll pitch yaw vector,
         * and with \f$  RPY(rpy) : \mathbb{R}^3 \mapsto SO(3) \f$ the function implemented
         * in the Rotation::RPY method, this method returns the right-trivialized partial
         * derivative of Rotation::RPY, i.e. :
         * \f[
         *    (RPY(rpy) \frac{d}{d t}\frac{\partial RPY(rpy)}{\partial rpy})^\vee
         * \f]
         */
        static Matrix3x3 RPYRightTrivializedDerivativeRateOfChange(const double roll, const double pitch, const double yaw, const double rollDot, const double pitchDot, const double yawDot);

        /**
         * Return the inverse of the right-trivialized derivative of the RPY function.
         *
         * See RPYRightTrivializedDerivative for a detailed description of the method.
         *
         */
        static Matrix3x3 RPYRightTrivializedDerivativeInverse(const double roll, const double pitch, const double yaw);

        /**
         *  Return the rate of change of the inverse of the right-trivialized derivative of the RPY function.
         *
         * See RPYRightTrivializedDerivativeRateOfChange for a detailed description of the method.
         *
         */
        static Matrix3x3 RPYRightTrivializedDerivativeInverseRateOfChange(const double roll, const double pitch, const double yaw, const double rollDot, const double pitchDot, const double yawDot);

        /**
         * Return the right-trivialized derivative of the Quaternion function.
         *
         * If we indicate with \f$ quat \in \mathbb{Q} \f$ the quaternion,
         * and with \f$  QUAT(quat) : \mathbb{Q} \mapsto SO(3) \f$ the function implemented
         * in the Rotation::RotationFromQuaternion method, this method returns the right-trivialized partial
         * derivative of Rotation::RotationFromQuaternion, i.e. :
         * \f[
         *    (QUAT(quat) \frac{\partial QUAT(quat)}{\partial quat})^\vee
         * \f]
         */
        static MatrixFixSize<4, 3> QuaternionRightTrivializedDerivative(Vector4 quaternion);

        /**
         * Return the inverse of the right-trivialized derivative of the Quaternion function.
         *
         * @see QuaternionRightTrivializedDerivative for a detailed description of the method.
         *
         */
        static MatrixFixSize<3, 4> QuaternionRightTrivializedDerivativeInverse(Vector4 quaternion);


        /**
         * Return an identity rotation.
         *
         *
         */
        static Rotation Identity();

        /**
         * Construct a rotation matrix from the given unit quaternion representation
         *
         * The quaternion is expected to be ordered in the following way:
         * - \f$s \in \mathbb{R}\f$ the real part of the quaterion
         * - \f$r \in \mathbb{R}^3\f$ the imaginary part of the quaternion
         *
         * The returned rotation matrix is given by the following formula:
         * \f[
         *   R(s,r) = I_{3\times3} + 2s r^{\wedge} + 2{r^\wedge}^2,
         * \f]
         * where \f$ r^{\wedge} \f$ is the skew-symmetric matrix such that:
         * \f[
         *   r \times v = r^\wedge v
         * \f]
         *
         * @note the quaternion is normalized
         * @param quaternion a quaternion representing a rotation
         *
         * @return The rotation matrix
         */
        static Rotation RotationFromQuaternion(const iDynTree::Vector4& quaternion);

        /**
         * Get the left Jacobian of rotation matrix
         *
         * \f$ \omega \in \mathbb{R}^3 \f$ is the angular motion vector
         * \f$ [\omega_\times]: \mathbb{R}^n \to \mathfrak{so}(3) \f$ where \f$ \mathfrak{so}(3) \f$
         *  is the set of skew symmetric matrices or the Lie algebra of \f$ SO(3) \f$
         * \f[ J_{l_{SO(3)}} = \sum_{n = 0}^{\infty} \frac{1}{(n+1)!} [\omega_\times]^n  = (I_3 + \frac{1 - \text{cos}(||\omega||)}{||\omega||^{2}} [\omega _{\times}] + \frac{||\omega|| - \text{sin}(||\omega||)}{||\omega||^{3}} [\omega _{\times}]^{2} \f]
         *
         * When simplified further,
         * \f[ J_{l_{SO(3)}} = \frac{\text{sin}(||\omega||)}{||\omega||}I_3 + \frac{1 - \text{cos}(||\omega||)}{||\omega||} [\phi _{\times}] + \bigg(1 - \frac{\text{sin}(||\omega||)}{||\omega||}\bigg) \phi\phi^T \f]
         *
         * where \f$ \phi = \frac{\omega}{||\omega||} \f$
         *
         * @param[in] omega angular motion vector
         * @return \f$ 3 \times 3 \f$ left Jacobian matrix
         */
        static Matrix3x3 leftJacobian(const iDynTree::AngularMotionVector3& omega);

        /**
         * Get the left Jacobian inverse of rotation matrix
         *
         * \f$ \omega \in \mathbb{R}^3 \f$ is the angular motion vector
         * \f$ [\omega_\times]: \mathbb{R}^n \to \mathfrak{so}(3) \f$ where \f$ \mathfrak{so}(3) \f$
         *  is the set of skew symmetric matrices or the Lie algebra of \f$ SO(3) \f$
         * \f[ J^{-1} _{l _{SO(3)}} = \frac{||\omega||}{2} \text{cot} \bigg(\frac{||\omega||}{2}\bigg) I _3 + \bigg( 1 - \frac{||\omega||}{2} \text{cot} \bigg(\frac{||\omega||}{2}\bigg) \bigg) \phi \phi^T - \frac{||\omega||}{2} [\phi _{\times}] \f]
         *
         * where \f$ \phi = \frac{\omega}{||\omega||} \f$
         *
         * @param[in] omega angular motion vector
         * @return \f$ 3 \times 3 \f$ left Jacobian inverse matrix
         */
        static Matrix3x3 leftJacobianInverse(const iDynTree::AngularMotionVector3& omega);
        ///@}

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}
    };

    IDYNTREE_DEPRECATED_WITH_MSG("iDynTree::RotationRaw is deprecated, use iDynTree::Rotation") typedef Rotation RotationRaw;
}

#endif
