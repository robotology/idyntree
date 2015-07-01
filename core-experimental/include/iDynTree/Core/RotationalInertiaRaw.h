/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ROTATIONAL_INERTIA_RAW_H
#define IDYNTREE_ROTATIONAL_INERTIA_RAW_H

#include "IMatrix.h"

namespace iDynTree
{
    class PositionRaw;

    /**
     * Class providing the raw coordinates for a 3d inertia matrix.
     *
     * \ingroup iDynTreeCore
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class RotationalInertiaRaw: IMatrix
    {
        double m_data[9]; /** storage for the rotational inertia matrix:
                                   given that the inertia matrix is a 3x3 symmetric matrix,
                                   the ordering (row order or column order) is not influencing
                                   the storage of the matrix. */

    public:
        RotationalInertiaRaw();
        RotationalInertiaRaw(const double * in_data, const unsigned int in_rows, const unsigned int in_cols);
        RotationalInertiaRaw(const RotationalInertiaRaw & other);
        virtual ~RotationalInertiaRaw();

        /**
         * Reset to the zero the elements of this matrix.
         */
        void zero();

        /**
         * @name Matrix interface methods.
         * Methods exposing a matrix-like interface to MatrixDynSize.
         *
         * \warning Notice that using this methods you can damage the underlyng rotation matrix.
         *          In doubt, don't use them and rely on more high level functions.
         */
        ///@{
        double operator()(const unsigned int row, const unsigned int col) const;
        double& operator()(const unsigned int row, const unsigned int col);
        double getVal(const unsigned int row, const unsigned int col) const;
        bool setVal(const unsigned int row, const unsigned int col, const double new_el);
        unsigned int rows() const;
        unsigned int cols() const;
        ///@}

        double * data();
        const double * data() const;
    };
}

#endif /* IDYNTREE_ROTATIONAL_INERTIA_RAW_H */