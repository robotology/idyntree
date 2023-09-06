// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Optimizers/OsqpInterface.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iDynTree/EigenHelpers.h>

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Utils.h>

#include <cassert>
#include <ostream>

#include <OsqpEigen/OsqpEigen.h>

#include <vector>
#include <iterator>

namespace iDynTree {
    namespace optimization {

        class Triplet {
        public:
            size_t m_row, m_col;
            double m_value;
            const size_t &row() const {
                return m_row;
            }

            const size_t &col() const {
                return m_col;
            }

            const double& value() const {
                return m_value;
            }
        };

        class TripletIterator : public std::iterator<std::input_iterator_tag, Triplet> {
            Triplet m_triplet;
            std::shared_ptr<std::vector<size_t>> m_rowIndeces;
            std::shared_ptr<std::vector<size_t>> m_colIndeces;
            std::shared_ptr<MatrixDynSize> m_denseMatrix;
            size_t m_nnzIdentity, m_nnzIndex;
            bool m_addIdentity;

        public:
            TripletIterator(std::shared_ptr<std::vector<size_t>> rowIndeces,
                            std::shared_ptr<std::vector<size_t>> colIndeces,
                            std::shared_ptr<MatrixDynSize> denseMatrix,
                            bool addIdentityOnTop = false)
            : m_rowIndeces(rowIndeces)
            , m_colIndeces(colIndeces)
            , m_denseMatrix(denseMatrix)
            , m_nnzIdentity(0)
            , m_nnzIndex(0)
            , m_addIdentity(addIdentityOnTop)
            {
            }

            TripletIterator& operator++() {
                if (m_addIdentity && (m_nnzIdentity < m_denseMatrix->cols())) {
                    m_nnzIdentity++;
                } else {
                    m_nnzIndex++;
                }
                return *this;
            }

            bool  operator==(const TripletIterator& rhs) const {
                if (m_addIdentity) {
                    return ((rhs.m_nnzIndex == this->m_nnzIndex) && (rhs.m_nnzIdentity == this->m_nnzIdentity));
                } else {
                    return (rhs.m_nnzIndex == this->m_nnzIndex);
                }
            }

            bool operator!=(const TripletIterator& rhs) const {
                if (m_addIdentity) {
                    return ((rhs.m_nnzIndex != this->m_nnzIndex) || (rhs.m_nnzIdentity != this->m_nnzIdentity));
                } else {
                    return (rhs.m_nnzIndex != this->m_nnzIndex);
                }
            }

            Triplet* operator->() {
                return &(operator*());
            }

            Triplet& operator*() {
                if (m_addIdentity && (m_nnzIdentity < m_denseMatrix->cols())) {
                    m_triplet.m_row = m_nnzIdentity;
                    m_triplet.m_col = m_nnzIdentity;
                    m_triplet.m_value = 1.0;
                    return m_triplet;
                }

                assert(m_nnzIndex < m_colIndeces->size());

                m_triplet.m_row = m_rowIndeces->operator[](m_nnzIndex) + m_nnzIdentity;
                m_triplet.m_col = m_colIndeces->operator[](m_nnzIndex);
                unsigned int row = static_cast<unsigned int>(m_rowIndeces->operator[](m_nnzIndex));
                unsigned int col = static_cast<unsigned int>(m_triplet.m_col);
                m_triplet.m_value = m_denseMatrix->operator()(row, col);
                return m_triplet;
            }

            static TripletIterator begin(std::shared_ptr<std::vector<size_t>> rowIndeces,
                                         std::shared_ptr<std::vector<size_t>> colIndeces,
                                         std::shared_ptr<MatrixDynSize> denseMatrix,
                                         bool addIdentityOnTop = false) {
                TripletIterator m_begin(rowIndeces, colIndeces, denseMatrix, addIdentityOnTop);
                assert(rowIndeces->size() == colIndeces->size());
                return m_begin;
            }

            static TripletIterator end(std::shared_ptr<std::vector<size_t>> rowIndeces,
                                       std::shared_ptr<std::vector<size_t>> colIndeces,
                                       std::shared_ptr<MatrixDynSize> denseMatrix,
                                       bool addIdentityOnTop = false) {
                TripletIterator m_end(rowIndeces, colIndeces, denseMatrix, addIdentityOnTop);
                if (addIdentityOnTop) {
                    m_end.m_nnzIdentity = denseMatrix->cols();
                }
                m_end.m_nnzIndex = rowIndeces->size();
                return m_end;
            }

        };

        class DenseIterator : public std::iterator<std::input_iterator_tag, Triplet> {
            Triplet m_triplet;
            std::shared_ptr<MatrixDynSize> m_denseMatrix;
            size_t m_nnzIdentity, m_rowIndex, m_colIndex;
            bool m_addIdentity;

        public:
            DenseIterator(std::shared_ptr<MatrixDynSize> denseMatrix,
                          bool addIdentityOnTop = false)
            : m_denseMatrix(denseMatrix)
            , m_nnzIdentity(0)
            , m_rowIndex(0)
            , m_colIndex(0)
            , m_addIdentity(addIdentityOnTop)
            {
            }

            DenseIterator& operator++() {
                if (m_addIdentity && (m_nnzIdentity < m_denseMatrix->cols())) {
                    m_nnzIdentity++;
                } else {
                    if (m_rowIndex < m_denseMatrix->rows()) {
                        m_rowIndex++;
                    } else {
                        if (m_colIndex < m_denseMatrix->cols()) {
                            m_colIndex++;
                            m_rowIndex = 0;
                        }
                    }
                }
                return *this;
            }

            bool  operator==(const DenseIterator& rhs) const{
                if (m_addIdentity) {
                    return ((rhs.m_rowIndex == this->m_rowIndex) && (rhs.m_colIndex == this->m_colIndex) && (rhs.m_nnzIdentity == this->m_nnzIdentity));
                } else {
                    return ((rhs.m_rowIndex == this->m_rowIndex) && (rhs.m_colIndex == this->m_colIndex));
                }
            }

            bool operator!=(const DenseIterator& rhs) const {
                return !(operator==(rhs));
            }

            Triplet* operator->() {
                return &(operator*());
            }

            Triplet& operator*() {
                if (m_addIdentity && (m_nnzIdentity < m_denseMatrix->cols())) {
                    m_triplet.m_row = m_nnzIdentity;
                    m_triplet.m_col = m_nnzIdentity;
                    m_triplet.m_value = 1.0;
                    return m_triplet;
                }

                assert(m_rowIndex < m_denseMatrix->rows());
                assert(m_colIndex < m_denseMatrix->cols());

                m_triplet.m_row = m_rowIndex + m_nnzIdentity;
                m_triplet.m_col = m_colIndex;
                unsigned int row = static_cast<unsigned int>(m_rowIndex);
                unsigned int col = static_cast<unsigned int>(m_triplet.m_col);
                m_triplet.m_value = m_denseMatrix->operator()(row, col);
                return m_triplet;
            }

            static DenseIterator begin(std::shared_ptr<MatrixDynSize> denseMatrix,
                                       bool addIdentityOnTop = false) {
                DenseIterator m_begin(denseMatrix, addIdentityOnTop);
                return m_begin;
            }

            static DenseIterator end(std::shared_ptr<MatrixDynSize> denseMatrix,
                                     bool addIdentityOnTop = false) {
                DenseIterator m_end(denseMatrix, addIdentityOnTop);
                if (addIdentityOnTop) {
                    m_end.m_nnzIdentity = denseMatrix->cols();
                }
                m_end.m_rowIndex = denseMatrix->rows();
                m_end.m_colIndex = denseMatrix->cols();
                return m_end;
            }

        };


        class OsqpInterface::OsqpInterfaceImplementation {
        public:
            OsqpSettings settings, previousSettings;
            OsqpEigen::Solver solver;
            bool initialGuessSet = false;
            VectorDynSize constraintsUpperBounds, constraintsLowerBounds;
            VectorDynSize variablesLowerBounds, variablesUpperBounds; //these bounds needs to be added manually to the constraints
            bool hasBoxConstraints;
            std::shared_ptr<std::vector<size_t>> constraintNNZRows, constraintNNZCols;
            std::shared_ptr<std::vector<size_t>> hessianNNZRows, hessianNNZCols;
            VectorDynSize variablesBuffer, solutionBuffer, constraintsBuffer;
            VectorDynSize costGradient, iDynTreeInitialGuess;
            std::shared_ptr<MatrixDynSize> costHessian;
            std::shared_ptr<MatrixDynSize> constraintJacobian;
            unsigned int nv, nc, previous_nv, previous_nc;

            Eigen::SparseMatrix<double> eigenHessian;
            Eigen::SparseMatrix<double> eigenJacobian;
            Eigen::VectorXd eigenGradient, eigenInitialGuess, unifiedLowerBounds, unifiedUpperBounds;
            Eigen::VectorXd eigenPrimalVariables, eigenDualVariables;
            bool alreadySolved = false;
            bool isLowerBounded = false;
            bool isUpperBounded = false;
            bool optionsAllowReoptimize = true;

            bool checkAndSetSetting() {
                if (!checkDoublesAreEqual(settings.rho, previousSettings.rho)) {
                    if (settings.rho <= 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "rho has to be >0");
                        return false;
                    }
                    solver.settings()->setRho(settings.rho);
                }

                if (!checkDoublesAreEqual(settings.sigma, previousSettings.sigma)) {
                    if (settings.sigma <= 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "sigma has to be >0");
                        return false;
                    }
                    solver.settings()->setSigma(settings.sigma);
                    optionsAllowReoptimize = false;
                }

                if (settings.max_iter != previousSettings.max_iter) {
                    if (settings.max_iter == 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "max_iter has to be >0");
                        return false;
                    }
                    solver.settings()->setMaxIteraction(static_cast<int>(settings.max_iter));
                }

                if (!checkDoublesAreEqual(settings.eps_abs, previousSettings.eps_abs)) {
                    if (settings.eps_abs < 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "eps_abs has to be >=0");
                        return false;
                    }
                    solver.settings()->setAbsoluteTolerance(settings.eps_abs);
                }

                if (!checkDoublesAreEqual(settings.eps_rel, previousSettings.eps_rel)) {
                    if (settings.eps_rel < 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "eps_rel has to be >=0");
                        return false;
                    }
                    solver.settings()->setRelativeTolerance(settings.eps_rel);
                }

                if (!checkDoublesAreEqual(settings.eps_prim_inf, previousSettings.eps_prim_inf)) {
                    if (settings.eps_prim_inf < 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "eps_prim_inf has to be >=0");
                        return false;
                    }
                    solver.settings()->setPrimalInfeasibilityTollerance(settings.eps_prim_inf);
                }

                if (!checkDoublesAreEqual(settings.eps_dual_inf, previousSettings.eps_dual_inf)) {
                    if (settings.eps_dual_inf < 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "eps_dual_inf has to be >=0");
                        return false;
                    }
                    solver.settings()->setDualInfeasibilityTollerance(settings.eps_dual_inf);
                }

                if (!checkDoublesAreEqual(settings.alpha, previousSettings.alpha)) {
                    if ((settings.alpha <= 0) || (settings.alpha >=2)) {
                        reportError("OsqpSettings", "checkAndSetSetting", "alpha has to be in the range (0,2).");
                        return false;
                    }
                    solver.settings()->setAlpha(settings.alpha);
                }

                if (settings.linsys_solver != previousSettings.linsys_solver) {
                    solver.settings()->setLinearSystemSolver(static_cast<int>(settings.linsys_solver));
                    optionsAllowReoptimize = false;
                }

                if (!checkDoublesAreEqual(settings.delta, previousSettings.delta)) {
                    if (settings.delta <= 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "delta has to be >0");
                        return false;
                    }
                    solver.settings()->setDelta(settings.delta);
                }

                if (settings.polish != previousSettings.polish) {
                    solver.settings()->setPolish(settings.polish);
                }

                if (settings.polish_refine_iter != previousSettings.polish_refine_iter) {
                    if (settings.polish_refine_iter == 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "polish_refine_iter has to be >0");
                        return false;
                    }
                    solver.settings()->setPolishRefineIter(static_cast<int>(settings.polish_refine_iter));
                }

                if (settings.verbose != previousSettings.verbose) {
                    solver.settings()->setVerbosity(settings.verbose);
                }

                if (settings.scaled_termination != previousSettings.scaled_termination) {
                    solver.settings()->setScaledTerimination(settings.scaled_termination);
                }

                if (settings.check_termination != previousSettings.check_termination) {
                    solver.settings()->setCheckTermination(static_cast<int>(settings.check_termination));
                }

                if (settings.warm_start != previousSettings.warm_start) {
                    solver.settings()->setWarmStart(settings.warm_start);
                }

                if (settings.scaling != previousSettings.scaling) {
                    solver.settings()->setScaling(static_cast<int>(settings.scaling));
                    optionsAllowReoptimize = false;
                }

                if (settings.adaptive_rho != previousSettings.adaptive_rho) {
                    solver.settings()->setAdaptiveRho(settings.adaptive_rho);
                    optionsAllowReoptimize = false;
                }

                if (settings.adaptive_rho_interval != previousSettings.adaptive_rho_interval) {
                    solver.settings()->setAdaptiveRhoInterval(static_cast<int>(settings.adaptive_rho_interval));
                    optionsAllowReoptimize = false;
                }

                if (!checkDoublesAreEqual(settings.adaptive_rho_tolerance, previousSettings.adaptive_rho_tolerance)) {
                    if (settings.adaptive_rho_tolerance < 1) {
                        reportError("OsqpSettings", "checkAndSetSetting", "adaptive_rho_tolerance has to be >= 1");
                        return false;
                    }
                    solver.settings()->setAdaptiveRhoTolerance(settings.adaptive_rho_tolerance);
                    optionsAllowReoptimize = false;
                }

                if (!checkDoublesAreEqual(settings.adaptive_rho_fraction, previousSettings.adaptive_rho_fraction)) {
                    if (settings.adaptive_rho_fraction <= 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "adaptive_rho_fraction has to be > 0");
                        return false;
                    }
                    solver.settings()->setAdaptiveRhoFraction(settings.adaptive_rho_fraction);
                    optionsAllowReoptimize = false;
                }

                if (!checkDoublesAreEqual(settings.time_limit, previousSettings.time_limit)) {
                    if (settings.time_limit < 0) {
                        reportError("OsqpSettings", "checkAndSetSetting", "time_limit has to be >= 0");
                        return false;
                    }
                    solver.settings()->getSettings()->time_limit = static_cast<c_float>(settings.time_limit);
                }

                return true;
            }

            bool possibleReStart() {
                bool initialized = solver.isInitialized();
                bool sameVariables = (previous_nv == nv);
                bool sameConstraints = (previous_nc == nc);
                return alreadySolved && optionsAllowReoptimize && initialized && sameVariables && sameConstraints;
            }
        };

        OsqpInterface::OsqpInterface()
        : m_pimpl(new OsqpInterfaceImplementation)
        {
            assert(m_pimpl);
            m_problem = nullptr;
            m_pimpl->costHessian = std::make_shared<MatrixDynSize>();
            m_pimpl->constraintJacobian = std::make_shared<MatrixDynSize>();
            m_pimpl->constraintNNZRows = std::make_shared<std::vector<size_t>>();
            m_pimpl->constraintNNZCols = std::make_shared<std::vector<size_t>>();
            m_pimpl->hessianNNZRows = std::make_shared<std::vector<size_t>>();
            m_pimpl->hessianNNZCols = std::make_shared<std::vector<size_t>>();
        }

        OsqpInterface::~OsqpInterface()
        {
            if (m_pimpl) {
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool OsqpInterface::isAvailable() const
        {
            return true;
        }

        bool OsqpInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
        {
            if (!problem){
                reportError("OsqpInterface", "setProblem", "Empty problem pointer.");
                return false;
            }
            m_problem = problem;
            return true;
        }

        bool OsqpInterface::solve()
        {
            if (!m_problem) {
                reportError("OsqpInterface", "solve", "Optimization problem not set.");
                return false;
            }

            if (!m_problem->prepare()){
                reportError("OsqpInterface", "solve", "Error while preparing the optimization problem.");
                return false;
            }

            if (!(m_problem->info().costIsQuadratic()) || (m_problem->info().hasNonLinearConstraints())) {
                reportError("OsqpInterface", "solve", "The specified optimization problem cannot be solved by this solver. It needs to be a QP.");
                return false;
            }

            if (!(m_problem->info().hessianIsProvided())) {
                reportError("OsqpInterface", "solve", "Hessian must be provided. Only the cost hessian will be evaluated.");
                return false;
            }

            m_pimpl->nv = m_problem->numberOfVariables();

            m_pimpl->isLowerBounded = m_problem->getVariablesLowerBound(m_pimpl->variablesLowerBounds);
            m_pimpl->isUpperBounded = m_problem->getVariablesUpperBound(m_pimpl->variablesUpperBounds);

            if (m_pimpl->isLowerBounded || m_pimpl->isUpperBounded) {
                m_pimpl->hasBoxConstraints = true;
                if (!(m_pimpl->isLowerBounded)) {
                    m_pimpl->variablesLowerBounds.resize(m_pimpl->nv);
                    toEigen(m_pimpl->variablesLowerBounds).setConstant(minusInfinity());
                }
                if (!(m_pimpl->isUpperBounded)) {
                    m_pimpl->variablesUpperBounds.resize(m_pimpl->nv);
                    toEigen(m_pimpl->variablesUpperBounds).setConstant(plusInfinity());
                }
            } else {
                m_pimpl->hasBoxConstraints = false;
            }

            m_pimpl->nc = m_problem->numberOfConstraints();

            if (m_pimpl->hasBoxConstraints) {
                m_pimpl->nc += m_pimpl->nv;
            }

            if (!(m_problem->getConstraintsBounds(m_pimpl->constraintsLowerBounds, m_pimpl->constraintsUpperBounds))) {
                reportError("OsqpInterface", "solve", "Error while retrieving constraint bounds.");
                return false;
            }

            if (m_pimpl->variablesBuffer.size() != m_pimpl->nv) {
                m_pimpl->variablesBuffer.resize(m_pimpl->nv);
                m_pimpl->variablesBuffer.zero();
            }

            if (!(m_problem->setVariables(m_pimpl->variablesBuffer))) {
                reportError("OsqpInterface", "solve", "Error while setting to optimization variables to the problem.");
                return false;
            }

            m_pimpl->constraintsBuffer.resize(m_pimpl->nc);

            if (!(m_problem->evaluateConstraints(m_pimpl->constraintsBuffer))) {
                reportError("OsqpInterface", "solve", "Error while evaluating constraints.");
                return false;
            }

            m_pimpl->unifiedLowerBounds.resize(m_pimpl->nc);
            m_pimpl->unifiedUpperBounds.resize(m_pimpl->nc);
            if (m_pimpl->hasBoxConstraints) {
                m_pimpl->unifiedLowerBounds.head(m_pimpl->nv) = toEigen(m_pimpl->variablesLowerBounds);
                m_pimpl->unifiedLowerBounds.tail(m_problem->numberOfConstraints()) =
                        toEigen(m_pimpl->constraintsLowerBounds) - toEigen(m_pimpl->constraintsBuffer); //remove eventual bias in the constraints. Infact, suppose that the constraint is written as Ax - b = 0, it is not enough to consider only the constraint jacobian A.

                m_pimpl->unifiedUpperBounds.head(m_pimpl->nv) = toEigen(m_pimpl->variablesUpperBounds);
                m_pimpl->unifiedUpperBounds.tail(m_problem->numberOfConstraints()) =
                        toEigen(m_pimpl->constraintsUpperBounds) - toEigen(m_pimpl->constraintsBuffer); //remove eventual bias in the constraints;
            } else {
                m_pimpl->unifiedLowerBounds = toEigen(m_pimpl->constraintsLowerBounds);
                m_pimpl->unifiedUpperBounds = toEigen(m_pimpl->constraintsUpperBounds);
            }

            if (!(m_problem->evaluateCostGradient(m_pimpl->costGradient))) {
                reportError("OsqpInterface", "solve", "Error while retrieving cost gradient.");
                return false;
            }

            m_pimpl->eigenGradient.resize(m_pimpl->nv);
            m_pimpl->eigenGradient = toEigen(m_pimpl->costGradient);

            if (m_problem->info().hasSparseHessian()) {
                if (!(m_problem->getHessianInfo(*(m_pimpl->hessianNNZRows), *(m_pimpl->hessianNNZCols)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving hessian sparsity structure.");
                    return false;
                }

                if (!(m_problem->evaluateCostHessian(*(m_pimpl->costHessian)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving cost hessian.");
                    return false;
                }

                TripletIterator beginIterator = TripletIterator::begin(m_pimpl->hessianNNZRows,
                                                                       m_pimpl->hessianNNZCols,
                                                                       m_pimpl->costHessian);

                TripletIterator endIterator = TripletIterator::end(m_pimpl->hessianNNZRows,
                                                                   m_pimpl->hessianNNZCols,
                                                                   m_pimpl->costHessian);

                m_pimpl->eigenHessian.resize(m_pimpl->nv, m_pimpl->nv);
                m_pimpl->eigenHessian.setFromTriplets(beginIterator, endIterator);
            } else {
                if (!(m_problem->evaluateCostHessian(*(m_pimpl->costHessian)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving cost hessian.");
                    return false;
                }

                DenseIterator beginIterator = DenseIterator::begin(m_pimpl->costHessian);

                DenseIterator endIterator = DenseIterator::end(m_pimpl->costHessian);

                m_pimpl->eigenHessian.resize(m_pimpl->nv, m_pimpl->nv);
                m_pimpl->eigenHessian.setFromTriplets(beginIterator, endIterator);
            }

            if (m_problem->info().hasSparseConstraintJacobian()) {
                if (!(m_problem->getConstraintsJacobianInfo(*(m_pimpl->constraintNNZRows), *(m_pimpl->constraintNNZCols)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving constraint jacobian sparsity structure.");
                    return false;
                }

                if (!(m_problem->evaluateConstraintsJacobian(*(m_pimpl->constraintJacobian)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving constraint jacobian.");
                    return false;
                }

                TripletIterator beginIterator = TripletIterator::begin(m_pimpl->constraintNNZRows,
                                                                       m_pimpl->constraintNNZCols,
                                                                       m_pimpl->constraintJacobian,
                                                                       m_pimpl->hasBoxConstraints);

                TripletIterator endIterator = TripletIterator::end(m_pimpl->constraintNNZRows,
                                                                   m_pimpl->constraintNNZCols,
                                                                   m_pimpl->constraintJacobian,
                                                                   m_pimpl->hasBoxConstraints);

                m_pimpl->eigenJacobian.resize(m_pimpl->nc, m_pimpl->nv);
                m_pimpl->eigenJacobian.setFromTriplets(beginIterator, endIterator);
            } else {
                if (!(m_problem->evaluateConstraintsJacobian(*(m_pimpl->constraintJacobian)))) {
                    reportError("OsqpInterface", "solve", "Error while retrieving constraint jacobian.");
                    return false;
                }

                DenseIterator beginIterator = DenseIterator::begin(m_pimpl->constraintJacobian,
                                                                   m_pimpl->hasBoxConstraints);

                DenseIterator endIterator = DenseIterator::end(m_pimpl->constraintJacobian,
                                                               m_pimpl->hasBoxConstraints);

                m_pimpl->eigenJacobian.resize(m_pimpl->nc, m_pimpl->nv);
                m_pimpl->eigenJacobian.setFromTriplets(beginIterator, endIterator);
            }

            if (!(m_pimpl->checkAndSetSetting())) {
                reportError("OsqpInterface", "solve", "The specified settings seems to be faulty.");
                return false;
            }

            if (m_pimpl->possibleReStart()) {

                if (!m_pimpl->solver.updateHessianMatrix(m_pimpl->eigenHessian)) {
                    reportError("OsqpInterface", "solve", "Error while updating the cost hessian.");
                    return false;
                }

                if (!m_pimpl->solver.updateGradient(m_pimpl->eigenGradient)) {
                    reportError("OsqpInterface", "solve", "Error while updating the cost gradient.");
                    return false;
                }

                if (!m_pimpl->solver.updateLinearConstraintsMatrix(m_pimpl->eigenJacobian)) {
                    reportError("OsqpInterface", "solve", "Error while updating the constraints jacobian.");
                    return false;
                }

                if (!m_pimpl->solver.updateBounds(m_pimpl->unifiedLowerBounds, m_pimpl->unifiedUpperBounds)) {
                    reportError("OsqpInterface", "solve", "Error while updating the constraints bounds.");
                    return false;
                }

                m_pimpl->initialGuessSet = m_problem->getGuess(m_pimpl->iDynTreeInitialGuess);
                m_pimpl->eigenInitialGuess.resize(m_pimpl->iDynTreeInitialGuess.size());
                m_pimpl->eigenInitialGuess = toEigen(m_pimpl->iDynTreeInitialGuess);
                if (m_pimpl->initialGuessSet) {
                    if (m_pimpl->eigenInitialGuess.size() != m_pimpl->nv) {
                        reportError("OsqpInterface", "solve", "The specified intial guess has dimension different from the number of variables.");
                        return false;
                    }

                    if (!(m_pimpl->solver.setPrimalVariable(m_pimpl->eigenInitialGuess))) {
                        reportError("OsqpInterface", "solve", "Error while setting the initial guess to the solver.");
                        return false;
                    }

                } else {
                    if (!(m_pimpl->solver.setPrimalVariable(m_pimpl->eigenPrimalVariables))) {
                        reportError("OsqpInterface", "solve", "Error while setting the initial guess from the previous run to the solver.");
                        return false;
                    }
                }

                if (!(m_pimpl->solver.setDualVariable(m_pimpl->eigenDualVariables))) {
                    reportError("OsqpInterface", "solve", "Error while setting the initial guess for dual variables to the solver.");
                    return false;
                }

            } else {
                if (m_pimpl->solver.isInitialized()) {
                    m_pimpl->solver.clearSolver();
                    m_pimpl->solver.data()->clearHessianMatrix();
                    m_pimpl->solver.data()->clearLinearConstraintsMatrix();
                }
                m_pimpl->alreadySolved = false;

                m_pimpl->solver.data()->setNumberOfVariables(static_cast<int>(m_pimpl->nv));

                m_pimpl->solver.data()->setNumberOfConstraints(static_cast<int>(m_pimpl->nc));

                if (!(m_pimpl->solver.data()->setHessianMatrix(m_pimpl->eigenHessian)))
                {
                    reportError("OsqpInterface", "solve", "Error while setting the hessian matrix during solver initialization.");
                    return false;
                }

                if (!(m_pimpl->solver.data()->setGradient(m_pimpl->eigenGradient))) {
                    reportError("OsqpInterface", "solve", "Error while setting the cost gradient during solver initialization.");
                    return false;
                }

                if (!(m_pimpl->solver.data()->setLinearConstraintsMatrix(m_pimpl->eigenJacobian))) {
                    reportError("OsqpInterface", "solve", "Error while setting the constraints jacobian during solver initialization.");
                    return false;
                }

                if (!(m_pimpl->solver.data()->setLowerBound(m_pimpl->unifiedLowerBounds))) {
                    reportError("OsqpInterface", "solve", "Error while setting the constraints lower bounds during solver initialization.");
                    return false;
                }

                if (!(m_pimpl->solver.data()->setUpperBound(m_pimpl->unifiedUpperBounds))) {
                    reportError("OsqpInterface", "solve", "Error while setting the constraints upper bounds during solver initialization.");
                    return false;
                }

                if (!(m_pimpl->solver.initSolver())) {
                    reportError("OsqpInterface", "solve", "Failed to initialize solver.");
                    return false;
                }

                m_pimpl->initialGuessSet = m_problem->getGuess(m_pimpl->iDynTreeInitialGuess);
                m_pimpl->eigenInitialGuess.resize(m_pimpl->iDynTreeInitialGuess.size());
                m_pimpl->eigenInitialGuess = toEigen(m_pimpl->iDynTreeInitialGuess);
                if (m_pimpl->initialGuessSet) {
                    if (m_pimpl->eigenInitialGuess.size() != m_pimpl->nv) {
                        reportError("OsqpInterface", "solve", "The specified intial guess has dimension different from the number of variables.");
                        return false;
                    }

                    if (!(m_pimpl->solver.setPrimalVariable(m_pimpl->eigenInitialGuess))) {
                        reportError("OsqpInterface", "solve", "Error while setting the initial guess to the solver.");
                        return false;
                    }
                }
            }

            if (!(m_pimpl->solver.isInitialized())) {
                reportError("OsqpInterface", "solve", "The solver does not seem to be initialized correctly.");
                return false;
            }

            if (!(m_pimpl->solver.solve())) {
                reportError("OsqpInterface", "solve", "Error while solving the QP.");
                return false;
            }

            m_pimpl->eigenPrimalVariables = m_pimpl->solver.getSolution();
            m_pimpl->solutionBuffer.resize(m_pimpl->nv);
            toEigen(m_pimpl->solutionBuffer) = m_pimpl->eigenPrimalVariables;
            m_pimpl->solver.getDualVariable(m_pimpl->eigenDualVariables);

            m_pimpl->alreadySolved = true;
            m_pimpl->initialGuessSet = false;
            m_pimpl->previous_nv = m_pimpl->nv;
            m_pimpl->previous_nc = m_pimpl->nc;
            m_pimpl->previousSettings = m_pimpl->settings;
            m_pimpl->optionsAllowReoptimize = true;
            return true;
        }

        bool OsqpInterface::getPrimalVariables(VectorDynSize &primalVariables)
        {
            if (!(m_pimpl->alreadySolved)) {
                reportError("OsqpInterface", "getPrimalVariables", "First you have to call the method solve once.");
                return false;
            }
            primalVariables = m_pimpl->solutionBuffer;
            return true;
        }

        bool OsqpInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
        {
            if (!(m_pimpl->alreadySolved)) {
                reportError("OsqpInterface", "getDualVariables", "First you have to call the method solve once.");
                return false;
            }

            constraintsMultipliers.resize(m_pimpl->nc);
            lowerBoundsMultipliers.resize(m_pimpl->nv);
            upperBoundsMultipliers.resize(m_pimpl->nv);

            if (!(m_pimpl->hasBoxConstraints)) {
                lowerBoundsMultipliers.zero();
                upperBoundsMultipliers.zero();
                constraintsMultipliers = constraintsMultipliers;
                return true;
            }

            toEigen(constraintsMultipliers) = m_pimpl->eigenDualVariables.tail(m_pimpl->nc);

            if (!(m_pimpl->isLowerBounded)) {
                lowerBoundsMultipliers.zero();
                toEigen(upperBoundsMultipliers) = m_pimpl->eigenDualVariables.head(m_pimpl->nv);
                return true;
            }

            if (!(m_pimpl->isUpperBounded)) {
                upperBoundsMultipliers.zero();
                toEigen(lowerBoundsMultipliers) = m_pimpl->eigenDualVariables.head(m_pimpl->nv);
                return true;
            }

            unsigned int iDynTreeIndex;
            double distanceToUpper, distanceToLower;
            for (Eigen::Index i = 0; i < m_pimpl->nv; ++i) {
                iDynTreeIndex = static_cast<unsigned int>(i);
                bool equalityConstraint = checkDoublesAreEqual(m_pimpl->variablesLowerBounds(iDynTreeIndex), m_pimpl->variablesUpperBounds(iDynTreeIndex), 1E-6);
                bool boundsNotActive = checkDoublesAreEqual(m_pimpl->eigenDualVariables(i), 0.0, 1E-6);
                if (equalityConstraint || boundsNotActive) {
                    lowerBoundsMultipliers(iDynTreeIndex) = m_pimpl->eigenDualVariables(i);
                    upperBoundsMultipliers(iDynTreeIndex) = m_pimpl->eigenDualVariables(i);
                } else {
                    distanceToLower = std::abs(m_pimpl->eigenPrimalVariables(i) - m_pimpl->variablesLowerBounds(iDynTreeIndex));
                    distanceToUpper = std::abs(m_pimpl->eigenPrimalVariables(i) - m_pimpl->variablesUpperBounds(iDynTreeIndex));
                    if (distanceToLower < distanceToUpper) {
                        lowerBoundsMultipliers(iDynTreeIndex) = m_pimpl->eigenDualVariables(i);
                        upperBoundsMultipliers(iDynTreeIndex) = 0.0;
                    } else {
                        lowerBoundsMultipliers(iDynTreeIndex) = 0.0;
                        upperBoundsMultipliers(iDynTreeIndex) = m_pimpl->eigenDualVariables(i);
                    }
                }
            }
            return true;
        }

        bool OsqpInterface::getOptimalCost(double &optimalCost)
        {
            if (!(m_pimpl->alreadySolved)) {
                reportError("OsqpInterface", "getOptimalCost", "First you have to call the method solve once.");
                return false;
            }

            if (!(m_problem->setVariables(m_pimpl->solutionBuffer))) {
                reportError("OsqpInterface", "getOptimalCost", "Error while setting to optimization variables to the problem.");
                return false;
            }

            if (!(m_problem->evaluateCostFunction(optimalCost))) {
                reportError("OsqpInterface", "solve", "Error while evaluating cost function.");
                return false;
            }

            return true;
        }

        bool OsqpInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
        {
            if (!(m_pimpl->alreadySolved)) {
                reportError("OsqpInterface", "getOptimalConstraintsValues", "First you have to call the method solve once.");
                return false;
            }

            if (!(m_problem->setVariables(m_pimpl->solutionBuffer))) {
                reportError("OsqpInterface", "getOptimalConstraintsValues", "Error while setting to optimization variables to the problem.");
                return false;
            }

            if (!(m_problem->evaluateConstraints(constraintsValues))) {
                reportError("OsqpInterface", "solve", "Error while evaluating cost function.");
                return false;
            }

            return true;
        }

        double OsqpInterface::minusInfinity()
        {
            return -OsqpEigen::INFTY;
        }

        double OsqpInterface::plusInfinity()
        {
            return OsqpEigen::INFTY;
        }

        OsqpSettings &OsqpInterface::settings()
        {
            return m_pimpl->settings;
        }

    }
}


