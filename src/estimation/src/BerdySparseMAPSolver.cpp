/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include "iDynTree/Estimation/BerdySparseMAPSolver.h"
#include "iDynTree/Estimation/BerdyHelper.h"

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <Eigen/SparseCore>
#include<Eigen/SparseQR>
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

        // Simulated y1 vector
        iDynTree::VectorDynSize simulatedMeasurementsFromMAPSolution;

        // Decomposition buffers
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > covarianceDynamicsPriorInverseDecomposition;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > covarianceDynamicsAPosterioriInverseDecomposition;

        // TODO: Guess having a separate buffers for berdy task1 solving is faster than changing the matrix sizes
        // Double check this

        // Task1 measurements
        iDynTree::VectorDynSize task1_measurements;

        // Priors on regularization, measurements and dynamics constraints
        // These variables should be filled in the "initialization" phase
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_priorDynamicsConstraintsCovarianceInverse; // Sigma_D^-1
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_priorDynamicsRegularizationCovarianceInverse; // Sigma_d^-1
        iDynTree::VectorDynSize task1_priorDynamicsRegularizationExpectedValue; // mu_d
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_priorMeasurementsCovarianceInverse; // Sigma_y^-1

        // The following variables are mainly buffers to contain intermediate results

        // These matrices contain the elements representing the berdy system
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_dynamicsConstraintsMatrix;
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_measurementsMatrix;
        iDynTree::VectorDynSize task1_dynamicsConstraintsBias;
        iDynTree::VectorDynSize task1_measurementsBias;

        // Expected value and variance of the prior on the dynamics
        iDynTree::VectorDynSize task1_expectedDynamicsPrior;
        Eigen::SparseMatrix<double, Eigen::ColMajor> task1_covarianceDynamicsPriorInverse;

        iDynTree::VectorDynSize task1_expectedDynamicsPriorRHS;

        // Expected value and variance of the a-posteriori on the dynamics
        iDynTree::VectorDynSize task1_expectedDynamicsAPosteriori;
        Eigen::SparseMatrix<double, Eigen::ColMajor> task1_covarianceDynamicsAPosterioriInverse;
        iDynTree::VectorDynSize task1_expectedDynamicsAPosterioriRHS;

        // Decomposition buffers
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > task1_covarianceDynamicsPriorInverseDecomposition;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > task1_covarianceDynamicsAPosterioriInverseDecomposition;

        // Task1 direct solution variables
        Eigen::SparseMatrix<double, Eigen::ColMajor> task1_directSolutionMeasurementMatrix;
        Eigen::SparseQR<Eigen::SparseMatrix<double, Eigen::ColMajor>, Eigen::NaturalOrdering<int> > task1_directSolutionMeasurementMatrixInverseDecomposition;


        BerdySparseMAPSolverPimpl(BerdyHelper& berdyHelper)
        : berdy(berdyHelper)
        , valid(false)
        {
            initialize();
        }

        bool initialize();
        void initializeTask1Buffers();
        void computeMAP(bool computePermutation);
        void computeTask1MAP(bool computePermutation);
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

    // New methods with additional bool flag for task1
    void BerdySparseMAPSolver::setDynamicsConstraintsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance,
                                                                     const bool& task1)
    {
        assert(m_pimpl);

        if (task1) {
            assert(covariance.rows() == m_pimpl->task1_priorDynamicsConstraintsCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->task1_priorDynamicsConstraintsCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->task1_priorDynamicsConstraintsCovarianceInverse);
        }
        else {
            assert(covariance.rows() == m_pimpl->priorDynamicsConstraintsCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->priorDynamicsConstraintsCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorDynamicsConstraintsCovarianceInverse);
        }

    }

    void BerdySparseMAPSolver::setDynamicsRegularizationPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance,
                                                                        const bool& task1)
    {
        assert(m_pimpl);

        if (task1) {
            assert(covariance.rows() == m_pimpl->task1_priorDynamicsRegularizationCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->task1_priorDynamicsRegularizationCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->task1_priorDynamicsRegularizationCovarianceInverse);
        }
        else {
            assert(covariance.rows() == m_pimpl->priorDynamicsRegularizationCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->priorDynamicsRegularizationCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorDynamicsRegularizationCovarianceInverse);
        }
    }

    void BerdySparseMAPSolver::setDynamicsRegularizationPriorExpectedValue(const iDynTree::VectorDynSize& expectedValue,
                                                                           const bool& task1)
    {
        assert(m_pimpl);

        if (task1) {
            assert(expectedValue.size() == m_pimpl->task1_priorDynamicsRegularizationExpectedValue.size());
            m_pimpl->task1_priorDynamicsRegularizationExpectedValue = expectedValue;
        }
        else {
            assert(expectedValue.size() == m_pimpl->priorDynamicsRegularizationExpectedValue.size());
            m_pimpl->priorDynamicsRegularizationExpectedValue = expectedValue;
        }
    }

    void BerdySparseMAPSolver::setMeasurementsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance,
                                                              const bool& task1)
    {
        assert(m_pimpl);

        if (task1) {
            assert(covariance.rows() == m_pimpl->task1_priorMeasurementsCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->task1_priorMeasurementsCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->task1_priorMeasurementsCovarianceInverse);
        }
        else {
            assert(covariance.rows() == m_pimpl->priorMeasurementsCovarianceInverse.rows()
                   && covariance.columns() == m_pimpl->priorMeasurementsCovarianceInverse.columns());
            BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::invertSparseMatrix(covariance, m_pimpl->priorMeasurementsCovarianceInverse);
        }
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

    void BerdySparseMAPSolver::updateEstimateInformationFloatingBase(const Transform& baseTransform,
                                                                     const iDynTree::JointPosDoubleArray& jointsConfiguration,
                                                                     const iDynTree::JointDOFsDoubleArray& jointsVelocity,
                                                                     const FrameIndex floatingFrame,
                                                                     const Vector3& bodyAngularVelocityOfSpecifiedFrame,
                                                                     const iDynTree::VectorDynSize& measurements,
                                                                     bool& task1)
    {
        assert(m_pimpl);

        m_pimpl->jointsConfiguration = jointsConfiguration;
        m_pimpl->jointsVelocity = jointsVelocity;

        if (task1) {
            m_pimpl->task1_measurements = measurements;
        }
        else {
            m_pimpl->measurements = measurements;
        }

        m_pimpl->berdy.updateKinematicsFromFloatingBase(baseTransform, m_pimpl->jointsConfiguration, m_pimpl->jointsVelocity,
                                                        floatingFrame, bodyAngularVelocityOfSpecifiedFrame);
    }

    bool BerdySparseMAPSolver::doEstimate(const bool& task1)
    {
        assert(m_pimpl);
        using iDynTree::toEigen;
#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(false);
#endif
        bool computePermutation = false;

        if (task1) {
            m_pimpl->computeTask1MAP(computePermutation);
        }
        else {
            m_pimpl->computeMAP(computePermutation);
        }

#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(true);
#endif
        return true;
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

    const iDynTree::VectorDynSize& BerdySparseMAPSolver::getSimulatedMeasurementVector(iDynTree::VectorDynSize& simulatedy, bool task1)
    {

        if(task1) {

            simulatedy = m_pimpl->simulatedMeasurementsFromMAPSolution;
        }

    }


    void BerdySparseMAPSolver::getLastEstimate(iDynTree::VectorDynSize& lastEstimate, const bool task1) const
    {
        assert(m_pimpl);

        if (task1) {
            lastEstimate = m_pimpl->task1_expectedDynamicsAPosteriori;
        }
        else {
            lastEstimate = m_pimpl->expectedDynamicsAPosteriori;
        }
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

    // Task1 MAP Computation
    void BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::computeTask1MAP(bool computePermutation)
    {
        /*
         * Get task1 berdy matrices
         */
        berdy.getBerdyMatrices(task1_dynamicsConstraintsMatrix,
                               task1_dynamicsConstraintsBias,
                               task1_measurementsMatrix,
                               task1_measurementsBias,
                               true);

        // This is a partial MAP solution considering only the task1 measurements equation  Y1 * d1 + bd1 = y1

        task1_covarianceDynamicsAPosterioriInverse = toEigen(task1_measurementsMatrix).transpose() * toEigen(task1_priorMeasurementsCovarianceInverse) * toEigen(task1_measurementsMatrix);

        // decompose m_covarianceDynamicsAPosterioriInverse
        if (computePermutation) {
            //        m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(toEigen(m_covarianceDynamicsAPosterioriInverse));
            task1_covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(task1_covarianceDynamicsAPosterioriInverse);
        }
        //    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.factorize(toEigen(m_covarianceDynamicsAPosterioriInverse));
        task1_covarianceDynamicsAPosterioriInverseDecomposition.factorize(task1_covarianceDynamicsAPosterioriInverse);

        // Final result: expected value of the whole-body dynamics, Eq. 11b
        toEigen(task1_expectedDynamicsAPosterioriRHS) = toEigen(task1_measurementsMatrix).transpose() * toEigen(task1_priorMeasurementsCovarianceInverse) * (toEigen(task1_measurements) - toEigen(task1_measurementsBias));
        toEigen(task1_expectedDynamicsAPosteriori) =
        task1_covarianceDynamicsAPosterioriInverseDecomposition.solve(toEigen(task1_expectedDynamicsAPosterioriRHS));

        // Compute simulated y1 from the solutions of MAP estimation
        simulatedMeasurementsFromMAPSolution.resize(berdy.getNrOfSensorsMeasurements(true));
        simulatedMeasurementsFromMAPSolution.zero();
        toEigen(simulatedMeasurementsFromMAPSolution) = toEigen(task1_measurementsMatrix).transpose() * toEigen(task1_expectedDynamicsAPosteriori);

//        std::cout << "Simulated y1 size : " << simulatedMeasurementsFromMAPSolution.size() << std::endl;
//        std::cout << "Simulated y1 : " << simulatedMeasurementsFromMAPSolution.toString() << std::endl;
//        std::cout << simulatedMeasurementsFromMAPSolution.toString() << std::endl;
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

        // Final result: covariance matrix of the whole-body dynamics, Eq. 11a
        //TODO: find a way to map to iDynTree::SparseMatrix
        covarianceDynamicsAPosterioriInverse = covarianceDynamicsPriorInverse;
        covarianceDynamicsAPosterioriInverse += toEigen(measurementsMatrix).transpose() * toEigen(priorMeasurementsCovarianceInverse) * toEigen(measurementsMatrix);

        // decompose m_covarianceDynamicsAPosterioriInverse
        if (computePermutation) {
            //        m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(toEigen(m_covarianceDynamicsAPosterioriInverse));
            covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(covarianceDynamicsAPosterioriInverse);
        }
        //    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.factorize(toEigen(m_covarianceDynamicsAPosterioriInverse));
        covarianceDynamicsAPosterioriInverseDecomposition.factorize(covarianceDynamicsAPosterioriInverse);

        // Final result: expected value of the whole-body dynamics, Eq. 11b
        toEigen(expectedDynamicsAPosterioriRHS) = (toEigen(measurementsMatrix).transpose() * toEigen(priorMeasurementsCovarianceInverse) * (toEigen(measurements) - toEigen(measurementsBias)) + covarianceDynamicsPriorInverse * toEigen(expectedDynamicsPrior));
        toEigen(expectedDynamicsAPosteriori) =
        covarianceDynamicsAPosterioriInverseDecomposition.solve(toEigen(expectedDynamicsAPosterioriRHS));
    }

    void BerdySparseMAPSolver::BerdySparseMAPSolverPimpl::initializeTask1Buffers()
    {
        // Resize task1 buffers
        size_t task1_numberOfDynVariables = berdy.getNrOfDynamicVariables(true);
        size_t task1_numberOfDynEquations = berdy.getNrOfDynamicEquations(true);
        size_t task1_numberOfMeasurements = berdy.getNrOfSensorsMeasurements(true);

        task1_measurements.resize(task1_numberOfMeasurements);
        task1_measurements.zero();

        task1_expectedDynamicsAPosteriori.resize(task1_numberOfDynVariables);
        task1_expectedDynamicsAPosteriori.zero();
        task1_covarianceDynamicsAPosterioriInverse.resize(task1_numberOfDynVariables, task1_numberOfDynVariables);

        task1_covarianceDynamicsPriorInverse.resize(task1_numberOfDynVariables, task1_numberOfDynVariables);
        task1_expectedDynamicsPrior.resize(task1_numberOfDynVariables);
        task1_expectedDynamicsPriorRHS.resize(task1_numberOfDynVariables);
        task1_expectedDynamicsAPosterioriRHS.resize(task1_numberOfDynVariables);
        task1_expectedDynamicsAPosterioriRHS.zero();

        // Resize task1 priors and set them to identity.
        // If a prior is specified in config file they will be cleared after
        iDynTree::Triplets identityTriplets;
        task1_priorDynamicsConstraintsCovarianceInverse.resize(task1_numberOfDynEquations, task1_numberOfDynEquations);
        identityTriplets.reserve(task1_numberOfDynEquations);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, task1_numberOfDynEquations);
        task1_priorDynamicsConstraintsCovarianceInverse.setFromTriplets(identityTriplets);

        task1_priorDynamicsRegularizationCovarianceInverse.resize(task1_numberOfDynVariables, task1_numberOfDynVariables);
        identityTriplets.clear();
        identityTriplets.reserve(task1_numberOfDynVariables);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, task1_numberOfDynVariables);
        task1_priorDynamicsRegularizationCovarianceInverse.setFromTriplets(identityTriplets);

        task1_priorMeasurementsCovarianceInverse.resize(task1_numberOfMeasurements, task1_numberOfMeasurements);
        identityTriplets.clear();
        identityTriplets.reserve(task1_numberOfMeasurements);
        identityTriplets.setDiagonalMatrix(0, 0, 1.0, task1_numberOfMeasurements);
        task1_priorMeasurementsCovarianceInverse.setFromTriplets(identityTriplets);

        task1_priorDynamicsRegularizationExpectedValue.resize(task1_numberOfDynVariables);
        task1_priorDynamicsRegularizationExpectedValue.zero();

        task1_directSolutionMeasurementMatrix.resize(task1_numberOfMeasurements, task1_numberOfDynVariables);

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

        // Call to initialize task1 buffers
        initializeTask1Buffers();

        berdy.updateKinematicsFromFixedBase(jointsConfiguration, jointsVelocity, berdy.dynamicTraversal().getBaseLink()->getIndex(), initialGravity);
        computeTask1MAP(true);
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
