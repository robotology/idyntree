// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SPATIAL_INERTIA_H
#define IDYNTREE_SPATIAL_INERTIA_H

#include <iDynTree/VectorFixSize.h>
#include <iDynTree/RotationalInertia.h>
#include <iDynTree/Twist.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Wrench.h>

namespace iDynTree
{
    /**
     * @brief Class representing a six dimensional inertia.
     *
     *
     * \ingroup iDynTreeCore
     */
    class SpatialInertia
    {
    protected:
        double m_mass; ///< Mass.
        double m_mcom[3]; ///< First moment of mass (i.e. mass * center of mass).
        RotationalInertia m_rotInertia; ///< Three dimensional rotational inertia.
    public:
        /**
         * Default constructor.
         * The data is not reset to zero for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        inline SpatialInertia() {}
        SpatialInertia(const double mass,
                       const Position& com,
                       const RotationalInertia & rotInertia);
        SpatialInertia(const SpatialInertia& other);

        // Operations on SpatialInertia
        static SpatialInertia combine(const SpatialInertia & op1,
                                      const SpatialInertia & op2);

        /**
         * Helper constructor-like function that takes mass, center of mass
         * and the rotational inertia expressed in the center of mass.
         *
         */
        void fromRotationalInertiaWrtCenterOfMass(const double mass, const Position& com, const RotationalInertia & rotInertia);


        /** multiplication operator
         *
         * overloading happens on proper classes
         *
         */


        /**
         * Getter functions
         *
         * \note for preserving consistency, no setters are implemented..
         *       if you want to modify a spatial inertia create a new one,
         *       and assign it to the spatial inertia that you want modify.
         *       Given that no memory allocation happens it should be still
         *       efficient.
         */
        double getMass() const;
        Position getCenterOfMass() const;
        const RotationalInertia& getRotationalInertiaWrtFrameOrigin() const;
        RotationalInertia getRotationalInertiaWrtCenterOfMass() const;

        /**
         * Multiplication function
         *
         */
        SpatialForceVector multiply(const SpatialMotionVector & op) const;

        /** reset to zero (i.e. the inertia of body with zero mass) the SpatialInertia */
        void zero();

        /**
         * @brief Get the SpatialInertia as a 6x6 matrix
         *
         * If \f$ m \in \mathbb{R} \f$ is the mass,
         * \f$ c \in \mathbb{R}^3 \f$ is the center of mass,
         * \f$ I \in \mathbb{R}^{3 \times 3} \f$ is the 3d inertia, and
         * \f$ 1_3 \in \mathbb{R}^{3 \times 3} \f$ is the 3d identity matrix this
         * method returns the \f$ \mathbb{M} \in \mathbb{R}^{6 \times 6} \f$ matrix such that:
         * \f[
         *  \mathbb{M} =
         *  \begin{bmatrix}
         *    m 1_3 & -m c \times \\
         *    m c \times & I
         *  \end{bmatrix}
         * \f].
         *
         * @note As all quantities in iDynTree, this inertia assumes the linear-angular serialization.
         */
        Matrix6x6 asMatrix() const;

        Twist applyInverse(const SpatialMomentum& mom) const;
        Matrix6x6 getInverse() const;

        // overloaded operators
        SpatialInertia  operator+(const SpatialInertia& other) const;
        SpatialForceVector operator*(const SpatialMotionVector &other) const;
        SpatialMomentum operator*(const Twist &other) const;
        Wrench operator*(const SpatialAcc &other) const;

        // Efficient operations

        /**
         * Return the bias wrench v.cross(M*v).
         *
         * Defining \f$ \mathbb{M} \f$ as this inertia, return
         * the bias wrench v.cross(M*v), defined in math as:
         * \f[
         *  \mathrm{v} \bar\times^* \mathbb{M} \mathrm{v} = \\
         *  \begin{bmatrix} \omega \times  & 0 \\
         *                  v \times  & \omega \times
         *  \end{bmatrix}
         *  \begin{bmatrix} m 1_3 & -mc\times \\
         *                  mc\times & I
         *  \end{bmatrix}
         *  \begin{bmatrix} v \\ \omega \end{bmatrix} = \\
         *  \begin{bmatrix}
         *   m \omega \times v - \omega \times ( m c \times \omega) \\
         *   m c \times ( \omega \times v ) + \omega \times I \omega
         *  \end{bmatrix}
         * \f]
         */
        Wrench biasWrench(const Twist & V) const;

        /**
         * @brief Return the derivative of the bias wrench with respect to the link 6D velocity.
         *
         * Defining \f$ \mathbb{M} \in \mathbb{R}^{6 \times 6} \f$ as this inertia, return the derivative
         * with respect to \f$ \mathrm{v} = \begin{bmatrix} v \\ \omega \end{bmatrix} \in \mathbb{R}^6 \f$
         * of the bias wrench \f$ \mathrm{v} \bar\times^* \mathbb{M} \mathrm{v} \f$ (i.e. v.cross(M*v)).
         *
         * The bias wrench is:
         * \f[
         *  \mathrm{v} \bar\times^* \mathbb{M} \mathrm{v} = \\
         *  \begin{bmatrix}
         *   m \omega \times v - \omega \times ( m c \times \omega) \\
         *   m c \times ( \omega \times v ) + \omega \times I \omega
         *  \end{bmatrix}
         * \f]
         *
         * So the derivative with respect to the twist V is :
         * \f[
         *  \partial_\mathrm{v} ( \mathrm{v} \bar\times^* \mathbb{M} \mathrm{v} ) = \\
         *  \begin{bmatrix}
         *   m \omega \times             &  - m v \times + ( m c \times \omega) \times - (\omega \times) (mc \times)   \\
         *   (m c \times) ( \omega \times)  & - (m c \times )(v \times) + \omega \times I - (I \omega) \times
         *  \end{bmatrix}
         * \f]
         */
        Matrix6x6 biasWrenchDerivative(const Twist & V) const;

        static SpatialInertia Zero();


        /**
         * \brief Get the Rigid Body Inertia as a vector of 10 inertial parameters.
         *
         * Return the rigid body inertia inertial parameters, defined as:
         *
         * | Elements | Symbol |  Description |
         * |:--------:|:-------:|:--------:|
         * |     0    | \f$ m \f$ | The mass of the rigid body |
         * |  1-3     |  \f$ m c \f$      | The first moment of mass of the rigid body |
         * |  4-9     | \f$ \mathop{vech}(I) \f$ | The 6 independent elements of the 3d inertia matrix, i.e. \f$ \begin{bmatrix} I_{xx} \\  I_{xy} \\  I_{xz} \\  I_{yy} \\  I_{yz} \\  I_{zz} \end{bmatrix} \f$ . |
         *
         * The first moment of mass is the center of mass (\f$ c \in \mathbb{R}^3 \f$ ) w.r.t. to the frame where this
         *  rigid body inertia is expressed multiplied by the rigid body mass \f$ m \f$.
         *
         * The 3d rigid body inertia \f$ I \in \mathbb{R}^{3 \times 3} \f$ is expressed with the orientation of the frame
         * in which this rigid body inertia is expressed, and with respect to the frame origin.
         *
         */
        Vector10 asVector() const;

        /**
         * \brief Set the Rigid Body Inertia from the inertial parameters in the vector.
         *
         * The serialization assumed in the inertialParams is the same used in the asVector method.
         */
        void fromVector(const Vector10 & inertialParams);

        /**
         * \brief Check if the Rigid Body Inertia is physically consistent.
         *
         * This method will check:
         *   * if the mass is positive,
         *   * if the 3d inertia at the COM is positive semidefinite,
         *     (semidefinite to cover also the case of the inertia of a point mass),
         *   * if the moment of inertia along the principal axes at the COM respect the triangle inequality.
         *
         * It will return true if all this check will pass, or false otherwise.
         *
         */
        bool isPhysicallyConsistent() const;

        /**
         * \brief Get the momentum inertial parameters regressor.
         *
         * Get the matrix
         * \f[
         *   Y(\mathrm{v}) \in \mathbb{R}^{6 \times 10}
         * \f]
         * such that:
         * \f[
         *   \mathbb{M} \mathrm{v} = Y(\mathrm{v}) \alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^{10} \f$ is the inertial parameters representation of \f$ \mathbb{M} \f$,
         * as returned by the asVector method.
         */
        static Matrix6x10 momentumRegressor(const iDynTree::Twist & v);

        /**
         * \brief Get the momentum derivative inertial parameters regressor.
         *
         * Get the matrix
         * \f[
         *   Y(\mathrm{v},a) \in \mathbb{R}^{6 \times 10}
         * \f]
         * such that:
         * \f[
         *   \mathbb{M} a + \mathrm{v} \overline{\times}^{*} \mathbb{M} \mathrm{v}   = Y(\mathrm{v}, a)\alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^{10} \f$ is the inertial parameters representation of \f$ \mathbb{M} \f$,
         * as returned by the asVector method.
         *
         * This is also the regressor of the net wrench acting on a rigid body.
         * As such, it is the building block of all other algorithms to compute dynamics
         * regressors.
         */
        static Matrix6x10 momentumDerivativeRegressor(const iDynTree::Twist & v,
                                                      const iDynTree::SpatialAcc & a);

        /**
         * \brief Get the momentum derivative inertial parameters regressor.
         *
         * Get the matrix
         * \f[
         *   Y(\mathrm{v},\mathrm{v}_r,a_r) \in \mathbb{R}^{6\times10}
         * \f]
         * such that:
         * \f[
         *   \mathbb{M} a_r + (\mathrm{v} \overline{\times}^{*} \mathbb{M} - \mathbb{M} \mathrm{v} \times) \mathrm{v}_r   = Y(\mathrm{v},\mathrm{v}_r,a_r)\alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^{10} \f$ is the inertial parameters representation of \f$ \mathbb{M} \f$, as returned by the
         * asVector method.
         *
         * Notice that if \f$ \mathrm{v} = \mathrm{v}_r \f$, this regressor reduces to the one computed by momentumDerivativeRegressor.
         * The main difference is that (assuming constant \f$ \mathbb{M} \f$) this regressor respect the passivity condition and
         * thus is the basic building block for building Slotine Li style regressors.
         *
         * For more on this, please check:
         *
         * Garofalo, G.; Ott, C.; Albu-Schaffer, A.,
         * "On the closed form computation of the dynamic matrices and their differentiations," in
         *  Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on
         * doi: 10.1109/IROS.2013.6696688
         * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6696688&isnumber=6696319
         *
         */
        static Matrix6x10 momentumDerivativeSlotineLiRegressor(const iDynTree::Twist & v,
                                                               const iDynTree::Twist & vRef,
                                                               const iDynTree::SpatialAcc & aRef);
    };
}

#endif
