/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_INERTIA_H
#define IDYNTREE_SPATIAL_INERTIA_H

#include <iDynTree/Core/SpatialInertiaRaw.h>

namespace iDynTree
{
    class Twist;
    class SpatialAcc;
    class SpatialMomentum;
    class Wrench;

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
         *   m \omega \times             &   ( m c \times \omega) \times - (\omega \times) (mc \times) \\
         *   (m c \times) ( \omega \times)  & \omega \times I - (I \omega) \times
         *  \end{bmatrix}
         * \f]
         */
        Matrix6x6 biasWrenchDerivative(const Twist & V) const;

        static SpatialInertia Zero();
    };
}

#endif