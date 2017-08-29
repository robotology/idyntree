/*
 * Copyright (C) 2016, 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro, Francesco Romano
 * email: silvio.traversaro@iit.it, francesco.romano@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IDYNTREE_BERDY_SPARSEMAPSOLVER_H
#define IDYNTREE_BERDY_SPARSEMAPSOLVER_H

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Indices.h>

namespace iDynTree {

    class BerdyHelper;
    class VectorDynSize;
    class JointPosDoubleArray;
    class JointDOFsDoubleArray;

    template <iDynTree::MatrixStorageOrdering ordering>
    class SparseMatrix;


    class BerdySparseMAPSolver
    {

        class BerdySparseMAPSolverPimpl;
        BerdySparseMAPSolverPimpl* m_pimpl;

    public:
        BerdySparseMAPSolver(BerdyHelper& berdyHelper);
        ~BerdySparseMAPSolver();

        void setDynamicsConstraintsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor> & covariance);
        void setDynamicsRegularizationPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance);
        void setDynamicsRegularizationPriorExpectedValue(const iDynTree::VectorDynSize& expectedValue);
        void setMeasurementsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance);

        const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& dynamicsConstraintsPriorCovarianceInverse() const; // Sigma_D^-1
        iDynTree::SparseMatrix<iDynTree::ColumnMajor>& dynamicsConstraintsPriorCovarianceInverse(); // Sigma_D^-1
        const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& dynamicsRegularizationPriorCovarianceInverse() const; // Sigma_d^-1
        iDynTree::SparseMatrix<iDynTree::ColumnMajor>& dynamicsRegularizationPriorCovarianceInverse(); // Sigma_d^-1
        const iDynTree::VectorDynSize& dynamicsRegularizationPriorExpectedValue() const; // mu_d
        iDynTree::VectorDynSize& dynamicsRegularizationPriorExpectedValue(); // mu_d
        const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& measurementsPriorCovarianceInverse() const; // Sigma_y^-1
        iDynTree::SparseMatrix<iDynTree::ColumnMajor>& measurementsPriorCovarianceInverse(); // Sigma_y^-1

        bool isValid();

        bool initialize();
        
        void updateEstimateInformationFixedBase(const JointPosDoubleArray& jointsConfiguration,
                                                const JointDOFsDoubleArray& jointsVelocity,
                                                const VectorDynSize& measurements);
        void updateEstimateInformationFixedBase(const JointPosDoubleArray& jointsConfiguration,
                                                const JointDOFsDoubleArray& jointsVelocity,
                                                const VectorDynSize& measurements,
                                                const Vector3& gravity);

        void updateEstimateInformationFloatingBase(const JointPosDoubleArray& jointsConfiguration,
                                                   const JointDOFsDoubleArray& jointsVelocity,
                                                   const VectorDynSize& measurements,
                                                   const FrameIndex& floatingFrame,
                                                   const Vector3& bodyAngularVelocityOfSpecifiedFrame);

        bool doEstimate();

        void getLastEstimate(iDynTree::VectorDynSize& lastEstimate) const;
        const iDynTree::VectorDynSize& getLastEstimate() const;


    };
}

#endif /* end of include guard: IDYNTREE_BERDY_SPARSEMAPSOLVER_H */
