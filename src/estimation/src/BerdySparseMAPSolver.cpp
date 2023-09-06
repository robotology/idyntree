// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include "iDynTree/BerdySparseMAPSolver.h"
#include "iDynTree/BerdyHelper.h"

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/SparseMatrix.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenSparseHelpers.h>

#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include <cassert>

namespace iDynTree {

    class BerdySparseMAPSolver::BerdySparseMAPSolverPimpl
    {
    public:
        iDynTree::BerdyHelper& berdy;

        bool valid;

        // Priors on regularization, measurements and dynamics constraints
        // These variables should be filled in the "initialization" phase
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorDynamicsConstraintsCovarianceInverse; // Sigma_D^-1
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorDynamicsRegularizationCovarianceInverse; // Sigma_d^-1
        iDynTree::VectorDynSize priorDynamicsRegularizationExpectedValue; // mu_d
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorMeasurementsCovarianceInverse; // Sigma_y^-1

        // The following variables are mainly buffers to contain intermediate results

        // These matrices contain the elements representing the berdy system
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintsMatrix;
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsMatrix;
        iDynTree::VectorDynSize dynamicsConstraintsBias;
        iDynTree::VectorDynSize measurementsBias;

        // Measurements vector and system state
        iDynTree::JointPosDoubleArray jointsConfiguration;
        iDynTree::JointDOFsDoubleArray jointsVelocity;
        iDynTree::VectorDynSize measurements;

        // Expected value and variance of the prior on the dynamics
        iDynTree::VectorDynSize expectedDynamicsPrior;
        Eigen::SparseMatrix<double, Eigen::ColMajor> covarianceDynamicsPriorInverse;

        iDynTree::VectorDynSize expectedDynamicsPriorRHS;

        // Expected value and variance of the a-posteriori on the dynamics
        iDynTree::VectorDynSize expectedDynamicsAPosteriori;
        Eigen::SparseMatrix<double, Eigen::ColMajor> covarianceDynamicsAPosterioriInverse;
        iDynTree::VectorDynSize expectedDynamicsAPosterioriRHS;

        // Decomposition buffers
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > covarianceDynamicsPriorInverseDecomposition;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > covarianceDynamicsAPosterioriInverseDecomposition;

        BerdySparseMAPSolverPimpl(BerdyHelper& berdyHelper)
        : berdy(berdyHelper)
        , valid(false)
        {
            initialize();
        }

        bool initialize();
        void computeMAP(bool computePermutation);
        static bool invertSparseMatrix(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>&in, iDynTree::SparseMatrix<iDynTree::ColumnMajor>& inverted);
    };

    BerdySparseMAPSolver::BerdySparseMAPSolver(BerdyHelper& berdyHelper)
    : m_pimpl(new BerdySparseMAPSolverPimpl(berdyHelper))
    {
        assert(m_pimpl);
    }

    BerdySparseMAPSolver::~BerdySparseMAPSolver()
    {
        assert(m_pimpl);
        delete m_pimpl;
    }

    void BerdySparseMAPSolver::setDynamicsConstraintsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance)
    {
        assert(m_pimpl);
        assert(m_pimpl->berdy.getOptions().berdyVariant!=BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES);
        assert(covariance.rows() == m_pimpl->priorDynamicsConstraintsCovarianceInverse.rows()
               && covariance.columns() == m_pimpl->priorDynamicsConstraintsCovarianceInverse.columns());
        BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorDynamicsConstraintsCovarianceInverse);
    }

    void BerdySparseMAPSolver::setDynamicsRegularizationPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance)
    {
        assert(m_pimpl);
        assert(covariance.rows() == m_pimpl->priorDynamicsRegularizationCovarianceInverse.rows()
               && covariance.columns() == m_pimpl->priorDynamicsRegularizationCovarianceInverse.columns());
        BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorDynamicsRegularizationCovarianceInverse);
    }

    void BerdySparseMAPSolver::setDynamicsRegularizationPriorExpectedValue(const iDynTree::VectorDynSize& expectedValue)
    {
        assert(m_pimpl);
        assert(expectedValue.size() == m_pimpl->priorDynamicsRegularizationExpectedValue.size());
        m_pimpl->priorDynamicsRegularizationExpectedValue = expectedValue;
    }

    void BerdySparseMAPSolver::setMeasurementsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance)
    {
        assert(m_pimpl);
        assert(covariance.rows() == m_pimpl->priorMeasurementsCovarianceInverse.rows()
               && covariance.columns() == m_pimpl->priorMeasurementsCovarianceInverse.columns());
        BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorMeasurementsCovarianceInverse);
    }

    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::dynamicsConstraintsPriorCovarianceInverse() const
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsConstraintsCovarianceInverse;
    }

    iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::dynamicsConstraintsPriorCovarianceInverse()
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsConstraintsCovarianceInverse;
    }

    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::dynamicsRegularizationPriorCovarianceInverse() const
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsRegularizationCovarianceInverse;
    }

    iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::dynamicsRegularizationPriorCovarianceInverse()
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsRegularizationCovarianceInverse;
    }

    const iDynTree::VectorDynSize& BerdySparseMAPSolver::dynamicsRegularizationPriorExpectedValue() const
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsRegularizationExpectedValue;
    }

    iDynTree::VectorDynSize& BerdySparseMAPSolver::dynamicsRegularizationPriorExpectedValue()
    {
        assert(m_pimpl);
        return m_pimpl->priorDynamicsRegularizationExpectedValue;
    }

    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::measurementsPriorCovarianceInverse() const
    {
        assert(m_pimpl);
        return m_pimpl->priorMeasurementsCovarianceInverse;
    }

    iDynTree::SparseMatrix<iDynTree::ColumnMajor>& BerdySparseMAPSolver::measurementsPriorCovarianceInverse()
    {
        assert(m_pimpl);
        return m_pimpl->priorMeasurementsCovarianceInverse;
    }

    bool BerdySparseMAPSolver::isValid()
    {
        assert(m_pimpl);
        return m_pimpl->valid;
    }

    bool BerdySparseMAPSolver::initialize()
    {
        assert(m_pimpl);
        bool init = m_pimpl->initialize();
        return init && m_pimpl->valid;
    }

    void BerdySparseMAPSolver::updateEstimateInformationFixedBase(const iDynTree::JointPosDoubleArray& jointsConfiguration,
                                                                  const iDynTree::JointDOFsDoubleArray& jointsVelocity,
                                                                  const FrameIndex fixedFrame,
                                                                  const Vector3& gravity,
                                                                  const iDynTree::VectorDynSize& measurements)
    {
        assert(m_pimpl);

        m_pimpl->jointsConfiguration = jointsConfiguration;
        m_pimpl->jointsVelocity = jointsVelocity;
        m_pimpl->measurements = measurements;

        m_pimpl->berdy.updateKinematicsFromFixedBase(m_pimpl->jointsConfiguration, m_pimpl->jointsVelocity, fixedFrame, gravity);
    }

    void BerdySparseMAPSolver::updateEstimateInformationFloatingBase(const iDynTree::JointPosDoubleArray& jointsConfiguration,
                                                                     const iDynTree::JointDOFsDoubleArray& jointsVelocity,
                                                                     const FrameIndex floatingFrame,
                                                                     const Vector3& bodyAngularVelocityOfSpecifiedFrame,
                                                                     const iDynTree::VectorDynSize& measurements)
    {
        assert(m_pimpl);

        m_pimpl->jointsConfiguration = jointsConfiguration;
        m_pimpl->jointsVelocity = jointsVelocity;
        m_pimpl->measurements = measurements;

        m_pimpl->berdy.updateKinematicsFromFloatingBase(m_pimpl->jointsConfiguration, m_pimpl->jointsVelocity,
                                                        floatingFrame, bodyAngularVelocityOfSpecifiedFrame);
    }

    bool BerdySparseMAPSolver::doEstimate()
    {
        assert(m_pimpl);
        using iDynTree::toEigen;
#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(false);
#endif
        bool computePermutation = false;
        m_pimpl->computeMAP(computePermutation);

#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(true);
#endif
        return true;
    }

    void BerdySparseMAPSolver::getLastEstimate(iDynTree::VectorDynSize& lastEstimate) const
    {
        assert(m_pimpl);
        lastEstimate = m_pimpl->expectedDynamicsAPosteriori;
    }

    const iDynTree::VectorDynSize& BerdySparseMAPSolver::getLastEstimate() const
    {
        assert(m_pimpl);
        return m_pimpl->expectedDynamicsAPosteriori;
    }

    void BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::computeMAP(bool computePermutation)
    {
        /*
         * Get berdy matrices
         */
        berdy.getBerdyMatrices(dynamicsConstraintsMatrix,
                               dynamicsConstraintsBias,
                               measurementsMatrix,
                               measurementsBias);


        // Compute the maximum a posteriori probability
        // See Latella et al., "Whole-Body Human Inverse Dynamics with
        // Distributed Micro-Accelerometers, Gyros and Force Sensing" in Sensors, 2016

        // Intermediate quantities

        if(berdy.getOptions().berdyVariant!=BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES)
        {
            // Covariance matrix of the prior of the dynamics: var[p(d)], Eq. 10a
            //TODO: find a way to map to iDynTree::SparseMatrix
            covarianceDynamicsPriorInverse
            //    toEigen(m_intermediateQuantities.covarianceDynamicsPriorInverse)
            //Better to assign the "sum" before, and adding the product part later
            = toEigen(priorDynamicsRegularizationCovarianceInverse);

            covarianceDynamicsPriorInverse += toEigen(dynamicsConstraintsMatrix).transpose() * toEigen(priorDynamicsConstraintsCovarianceInverse) * toEigen(dynamicsConstraintsMatrix);

            // decompose m_covarianceDynamicsPriorInverse
            if (computePermutation) {
                covarianceDynamicsPriorInverseDecomposition.analyzePattern(covarianceDynamicsPriorInverse);
            }
            //    m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.factorize(toEigen(m_intermediateQuantities.covarianceDynamicsPriorInverse));
            covarianceDynamicsPriorInverseDecomposition.factorize(covarianceDynamicsPriorInverse);

            // Expected value of the prior of the dynamics: E[p(d)], Eq. 10b
            toEigen(expectedDynamicsPriorRHS) = toEigen(priorDynamicsRegularizationCovarianceInverse) * toEigen(priorDynamicsRegularizationExpectedValue) - toEigen(dynamicsConstraintsMatrix).transpose() * toEigen(priorDynamicsConstraintsCovarianceInverse) * toEigen(dynamicsConstraintsBias);

            toEigen(expectedDynamicsPrior) =
            covarianceDynamicsPriorInverseDecomposition.solve(toEigen(expectedDynamicsPriorRHS));
        }
        else
        {
            // Modified eq. 10a and 10b without the dynamics constraints
            covarianceDynamicsPriorInverse = toEigen(priorDynamicsRegularizationCovarianceInverse);
            expectedDynamicsPrior = priorDynamicsRegularizationExpectedValue;
        }
        
        // Final result: covariance matrix of the whole-body dynamics, Eq. 11a
        //TODO: find a way to map to iDynTree::SparseMatrix
        covarianceDynamicsAPosterioriInverse = toEigen(measurementsMatrix).transpose() * toEigen(priorMeasurementsCovarianceInverse) * toEigen(measurementsMatrix);
        covarianceDynamicsAPosterioriInverse += covarianceDynamicsPriorInverse;


        // decompose m_covarianceDynamicsAPosterioriInverse
        if (computePermutation) {
            //        m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(toEigen(m_covarianceDynamicsAPosterioriInverse));
            covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(covarianceDynamicsAPosterioriInverse);
        }
        //    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.factorize(toEigen(m_covarianceDynamicsAPosterioriInverse));
        covarianceDynamicsAPosterioriInverseDecomposition.factorize(covarianceDynamicsAPosterioriInverse);

        // Final result: expected value of the whole-body dynamics, Eq. 11b
        toEigen(expectedDynamicsAPosterioriRHS) = toEigen(measurementsMatrix).transpose() * toEigen(priorMeasurementsCovarianceInverse) * (toEigen(measurements) - toEigen(measurementsBias));
        toEigen(expectedDynamicsAPosterioriRHS) += covarianceDynamicsPriorInverse * toEigen(expectedDynamicsPrior);
        
        toEigen(expectedDynamicsAPosteriori) =
        covarianceDynamicsAPosterioriInverseDecomposition.solve(toEigen(expectedDynamicsAPosterioriRHS));
    }

    bool BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::initialize()
    {
        valid = false;
        if (!berdy.isValid()) return false;
        // Resize internal variables
        size_t numberOfDynVariables = berdy.getNrOfDynamicVariables();
        size_t numberOfDynEquations = berdy.getNrOfDynamicEquations();
        size_t numberOfMeasurements = berdy.getNrOfSensorsMeasurements();

        jointsConfiguration.resize(berdy.model());
        jointsConfiguration.zero();
        jointsVelocity.resize(berdy.model());
        jointsVelocity.zero();

        measurements.resize(numberOfMeasurements);
        measurements.zero();

        expectedDynamicsAPosteriori.resize(numberOfDynVariables);
        expectedDynamicsAPosteriori.zero();
        covarianceDynamicsAPosterioriInverse.resize(numberOfDynVariables, numberOfDynVariables);

        covarianceDynamicsPriorInverse.resize(numberOfDynVariables, numberOfDynVariables);
        expectedDynamicsPrior.resize(numberOfDynVariables);
        expectedDynamicsPriorRHS.resize(numberOfDynVariables);
        expectedDynamicsAPosterioriRHS.resize(numberOfDynVariables);
        expectedDynamicsAPosterioriRHS.zero();

        // Resize priors and set them to identity.
        // If a prior is specified in config file they will be cleared after
        iDynTree::Triplets identityTriplets;
        priorDynamicsConstraintsCovarianceInverse.resize(numberOfDynEquations, numberOfDynEquations);
        identityTriplets.reserve(numberOfDynEquations);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfDynEquations);
        priorDynamicsConstraintsCovarianceInverse.setFromTriplets(identityTriplets);

        priorDynamicsRegularizationCovarianceInverse.resize(numberOfDynVariables, numberOfDynVariables);
        identityTriplets.clear();
        identityTriplets.reserve(numberOfDynVariables);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfDynVariables);
        priorDynamicsRegularizationCovarianceInverse.setFromTriplets(identityTriplets);

        priorMeasurementsCovarianceInverse.resize(numberOfMeasurements, numberOfMeasurements);
        identityTriplets.clear();
        identityTriplets.reserve(numberOfMeasurements);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfMeasurements);
        priorMeasurementsCovarianceInverse.setFromTriplets(identityTriplets);

        priorDynamicsRegularizationExpectedValue.resize(numberOfDynVariables);
        priorDynamicsRegularizationExpectedValue.zero();

        Vector3 initialGravity;
        initialGravity.zero();
        initialGravity(2) = -9.81;

        berdy.updateKinematicsFromFixedBase(jointsConfiguration, jointsVelocity, berdy.dynamicTraversal().getBaseLink()->getIndex(), initialGravity);
        computeMAP(true);

        valid = true;
        return true;
    }


    bool BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>&in,
                                                                             iDynTree::SparseMatrix<iDynTree::ColumnMajor>& inverted)
    {
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > inversion;
        inversion.compute(iDynTree::toEigen(in));
        if (inversion.info() != Eigen::Success) return false;
        
        Eigen::SparseMatrix<double> identity(in.rows(), in.columns());
        identity.setIdentity();
        Eigen::SparseMatrix<double> result = inversion.solve(identity);
        iDynTree::Triplets invertedElements;
        invertedElements.reserve(result.nonZeros());

        for (Eigen::Index k = 0; k < result.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(result, k); it; ++it)
            {
                invertedElements.pushTriplet(iDynTree::Triplet(it.row(), it.col(), it.value()));
            }
        }
        inverted.setFromTriplets(invertedElements);
        return true;
    }
}
