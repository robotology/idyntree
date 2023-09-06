// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_ARTICULATED_BODY_INERTIA_H
#define IDYNTREE_ARTICULATED_BODY_INERTIA_H

#include <iDynTree/MatrixFixSize.h>
#include <string>

namespace iDynTree
{
    class SpatialInertia;
    class SpatialForceVector;
    class SpatialMotionVector;
    class Wrench;
    class SpatialAcc;

    /**
     * Class representing an Articulated Body Inertia.
     *
     * For more information on Articulated Body Inertia, please
     * check Featherstone 2008, Chapter 7 .
     *
     * Storage:
     * The symmetric articulated body inertia is stored as 3
     * 3x3 matrices :
     *  * the linearLinear is the top left submatrix
     *  * the linearAngular is the top right submatrix
     *  * the angularAngular is the bottom right submatrix
     *
     * The bottom left submatrix can be obtained as the
     * transpose of the linearAngular matrix.
     *
     *
     *
     * \warning This class is exposing for convenience the IMatrix interface.
     *          Notice that using this methods you can damage the underlyng articulated body inertia.
     *          In doubt, don't use them and rely on more high level functions.
     *
     * \ingroup iDynTreeCore
     */
    class ArticulatedBodyInertia
    {
    private:
        Matrix3x3 linearLinear;
        Matrix3x3 linearAngular;
        Matrix3x3 angularAngular;

    public:
        /**
         * The data is not reset to zeo for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        ArticulatedBodyInertia();

        /**
         * Constructor from a buffer of 36 doubles,
         * stored as a C-style array (i.e. row major).
         *
         */
        ArticulatedBodyInertia(const double* in_data,
                               const unsigned int in_rows,
                               const unsigned int in_cols);

        /**
         * Constructor from a Rigid Body Inertia.
         */
        ArticulatedBodyInertia(const iDynTree::SpatialInertia& rigidBodyInertia);

        /**
         * Copy constructor: create a ArticulatedBodyInertia from another ArticulatedBodyInertia.
         */
        ArticulatedBodyInertia(const ArticulatedBodyInertia & other);

        /**
         * Low level data getters.
         */
        Matrix3x3 & getLinearLinearSubmatrix();
        Matrix3x3 & getLinearAngularSubmatrix();
        Matrix3x3 & getAngularAngularSubmatrix();
        const Matrix3x3 & getLinearLinearSubmatrix() const;
        const Matrix3x3 & getLinearAngularSubmatrix() const;
        const Matrix3x3 & getAngularAngularSubmatrix() const;

        // Operations on SpatialInertia
        static ArticulatedBodyInertia combine(const ArticulatedBodyInertia & op1,
                                              const ArticulatedBodyInertia & op2);

        /**
         * Return the inverse of this ArticulatedBodyInertia
         * applied to the passed wrench.
         */
        SpatialAcc applyInverse(const Wrench & wrench) const;

        /**
         * Get the ArticulatedBodyInertia as a 6x6 matrix
         */
        Matrix6x6 asMatrix() const;

        /**
         * Get the inverse of the ArticulatedBodyInertia matrix.
         */
        Matrix6x6 getInverse() const;

        // overloaded operators
        ArticulatedBodyInertia  operator+(const ArticulatedBodyInertia& other) const;
        ArticulatedBodyInertia  operator-(const ArticulatedBodyInertia& other) const;
        ArticulatedBodyInertia& operator+=(const ArticulatedBodyInertia& other);
        SpatialForceVector operator*(const SpatialMotionVector &other) const;
        Wrench operator*(const SpatialAcc &other) const;


        /** reset to zero (i.e. the inertia of body with zero mass) the ArticulatedBodyInertia */
        void zero();

        // Static helpers

        /**
         * Build the ArticulatedInertia \f$U d^{-1} U^\top\f$ .
         * Used in the articulated body algorithm .
         */
        static ArticulatedBodyInertia ABADyadHelper(const SpatialForceVector & U, const double d);

        /**
         * Build the ArticulatedInertia dU inv_d U^\top + U d_inv_d U^\top + U inv_d dU^\top
         * Used in the linearization of the articulated body algorithm.
         */
        static ArticulatedBodyInertia ABADyadHelperLin(const SpatialForceVector &  U, const double   inv_d,
                                                       const SpatialForceVector & dU, const double d_inv_d);

    };
}

#endif
