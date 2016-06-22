/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_INERTIA_H
#define IDYNTREE_SPATIAL_INERTIA_H

#include <iDynTree/Core/SpatialInertiaRaw.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Wrench.h>

namespace iDynTree
{
    /**
     * Class representing a spatial inertia
     *
     *
     * Currently this class does not support semantics.
     *
     * \ingroup iDynTreeCore
     */
    class SpatialInertia: public SpatialInertiaRaw
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to zero for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        inline SpatialInertia() {};
        SpatialInertia(const double mass,
                       const PositionRaw & com,
                       const RotationalInertiaRaw & rotInertia);
        SpatialInertia(const SpatialInertiaRaw& other);
        SpatialInertia(const SpatialInertia& other);

        // Operations on SpatialInertia
        static SpatialInertia combine(const SpatialInertia & op1,
                                      const SpatialInertia & op2);

        // Get the SpatialInertia as a 6x6 matrix
        Matrix6x6 asMatrix() const;

        // overloaded operators
        SpatialInertia  operator+(const SpatialInertia& other) const;
        SpatialForceVector operator*(const SpatialMotionVector &other) const;
        SpatialMomentum operator*(const Twist &other) const;
        Wrench operator*(const SpatialAcc &other) const;

        // Efficient operations

        /**
         * Return the bias wrench V.cross(M*V).
         *
         * Defining \f$ M \f$ as this inertia, return
         * the bias wrench V.cross(M*V), defined in math as:
         * \f[
         *  V \bar\times^* M V = \\
         *  \begin{bmatrix} \omega \times  & 0 \\
         *                  v \times  & \omega \times
         *  \end{bmatrix}
         *  \begin{bmatrix} m & -mc\times \\
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
         * Return the derivative of the bias wrench with respect to the link twist.
         *
         * Defining \f$ M \f$ as this inertia, return the derivative
         * with respect to V  of the bias wrench V.cross(M*V).
         *
         * The bias wrench is:
         * \f[
         *  V \bar\times^* M V = \\
         *  \begin{bmatrix}
         *   m \omega \times v - \omega \times ( m c \times \omega) \\
         *   m c \times ( \omega \times v ) + \omega \times I \omega
         *  \end{bmatrix}
         * \f]
         *
         * So the derivative with respect to the twist V is :
         * \f[
         *  \partial_V ( V \bar\times^* M V ) = \\
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
         * |  2-4     |  \f$ m c \f$      | The first moment of mass of the rigid body |
         * |  5-9     | \f$ \operatorname{vech}(I_o) \f$ | The 6 indipendent elements of the 3d inertia matrix (\f$ I_{xx} I_{xy} I_{xz} I_{yy} I_{yz} I_{zz} \f$). |
         *
         * The first moment of mass is the center of mass (\f$ c \in \mathbb{R}^3 \f$ ) w.r.t. to the frame where this
         *  rigid body inertia is expressed multiplied by the rigid body mass \f$ m \f$.
         *
         * The 3d rigid body inertia \f$ I_o \in \mathbb{R}^{3 \times 3} \f$ is expressed with the orientation of the frame
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
         *   Y(v) \in \mathbb{R}^{6 \times 10}
         * \f]
         * such that:
         * \f[
         *   I v = Y(v)\alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^10 \f$ is the inertial parameters representation of \f$ I \f$ .
         */
        static Matrix6x10 momentumRegressor(const iDynTree::Twist & v);

        /**
         * \brief Get the momentum derivative inertial parameters regressor.
         *
         * Get the matrix
         * \f[
         *   Y(v,a) \in \mathbb{R}^{6 \times 10}
         * \f]
         * such that:
         * \f[
         *   I a + v \overline{\times}^{*} I v   = Y(v,a)\alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^10 \f$ is the inertial parameters representation of \f$ I \f$
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
         *   Y(v,v_r,a_r) \in \mathbb{R}^{6\times6}
         * \f]
         * such that:
         * \f[
         *   I a_r + (v \overline{\times}^{*} I - I v \times)  v_r   = Y(v,v_r,a_r)\alpha
         * \f]
         *
         * If \f$ \alpha \in \mathbb{R}^10 \f$ is the inertial parameters representation of \f$ I \f$
         *
         * Notice that if \f$ v = v_r \f$, this regressor reduces to the one computed by momentumDerivativeRegressor.
         * The main difference is that (assuming constant \f$ I \f$) this regressor respect the passivity condition and
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
