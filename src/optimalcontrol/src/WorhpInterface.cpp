/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Optimizers/WorhpInterface.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Triplets.h>
#include <worhp/worhp.h>
#include <algorithm>
#include <unordered_map>

using namespace iDynTree::optimization;

typedef struct {
    OptVar opt;
    Workspace wsp;
    Params par;
    Control cnt;
    int status;
} WorhpVariables;

class MatrixElement {

    bool prioritySign() const{
        int thisIndexDifference = static_cast<int>(row) - static_cast<int>(col);
        return (thisIndexDifference > 0) - (thisIndexDifference < 0); // +1 is the lower triangular part, 0 the diagonal, -1 the upper triangular part
    }

public:
    unsigned int row;
    unsigned int col;

    bool operator < (const MatrixElement& other) const {
        return this->col < other.col
                || (this->col == other.col && this->row < other.row);
    }

    bool lowerTriangularCompare(const MatrixElement& other) const {
        return prioritySign() > other.prioritySign()
                || (prioritySign() == other.prioritySign() && this->operator<(other));
    }
};

class WorhpInterface::WorhpInterfaceImplementation {
public:
    WorhpVariables worhp;
    std::vector<size_t> constraintsJacNNZRows, constraintsJacNNZCols, hessianNNZRows, hessianNNZCols;
    size_t hessianLowerTriangularNonZeros;
    iDynTree::VectorDynSize initialPoint;
    iDynTree::VectorDynSize variablesLowerBounds, variablesUpperBounds, constraintsLowerBounds, constraintsUpperBounds;
    iDynTree::VectorDynSize variablesBuffer, constraintsEvaluationBuffer, costGradientBuffer, constraintsMultipliersBuffer;
    iDynTree::MatrixDynSize constraintsJacobianBuffer, costHessianBuffer, constraintsHessianBuffer;
    std::vector<MatrixElement> orderedJacobianNonZeros, orderedHessianNonZeros;
    unsigned int previousNumberOfVariables, previousNumberOfConstraints, previousJacobianNonZeros, previousHessianNonZeros;
    std::unordered_map<std::string, bool> boolParamsBackup;
    std::unordered_map<std::string, int> intParamsBackup;
    std::unordered_map<std::string, double> doubleParamsBackup;
    bool sparseJacobian, sparseHessian;
    bool previouslySolved;
    bool firstTime;
    bool useApproximatedHessian;


    void resizeBuffers(unsigned int numberOfVariables, unsigned int numberOfConstraints) {
        variablesBuffer.resize(numberOfVariables);
        constraintsEvaluationBuffer.resize(numberOfConstraints);
        costGradientBuffer.resize(numberOfVariables);
        constraintsJacobianBuffer.resize(numberOfConstraints,numberOfVariables);
        constraintsJacobianBuffer.zero();
        costHessianBuffer.resize(numberOfVariables, numberOfVariables);
        costHessianBuffer.zero();
        constraintsHessianBuffer.resize(numberOfVariables, numberOfVariables);
        constraintsHessianBuffer.zero();
        constraintsMultipliersBuffer.resize(numberOfConstraints);
        initialPoint.resize(numberOfVariables);

    }

    bool possibleReOptimize(unsigned int newNumberOfVariables, unsigned int newNumberOfconstraints,
                            unsigned int newJacobianNonZeros, unsigned int newHessianNonZeros){
        bool sameVariables = (previousNumberOfVariables == newNumberOfVariables);
        bool sameConstraints = (previousNumberOfConstraints == newNumberOfconstraints);
        bool sameNNZJac = (previousJacobianNonZeros == newJacobianNonZeros);
        bool sameNNZHes = (previousHessianNonZeros == newHessianNonZeros);
        return previouslySolved && sameVariables && sameConstraints && sameNNZJac && sameNNZHes;
    }

    bool updateVariables(std::shared_ptr<iDynTree::optimization::OptimizationProblem> problem) {
        if (worhp.opt.newX) {
            Eigen::Map<const Eigen::VectorXd> variablesMap(worhp.opt.X, worhp.opt.n);
            iDynTree::toEigen(variablesBuffer) = variablesMap;
            if (!problem->setVariables(variablesBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to update variables.");
                previouslySolved = false;
                return false;
            }
        }
        return true;
    }

};


WorhpInterface::WorhpInterface()
    : m_pimpl(new WorhpInterfaceImplementation)
{
    WorhpPreInit(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
    CHECK_WORHP_VERSION;
    m_pimpl->previouslySolved = false;
    m_pimpl->firstTime = true;
    m_pimpl->useApproximatedHessian = false;
    /*
    * Parameter initialisation routine
    */
    InitParams(&(m_pimpl->worhp.status), &(m_pimpl->worhp.par));
}

WorhpInterface::~WorhpInterface()
{
    if (m_pimpl) {
        WorhpFree(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
        delete m_pimpl;
        m_pimpl = nullptr;
    }
}

bool WorhpInterface::isAvailable() const
{
    return true;
}

bool WorhpInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
{
    if (!problem){
        reportError("WorhpInterface", "setProblem", "Empty problem pointer.");
        return false;
    }
    m_problem = problem;
    m_pimpl->previouslySolved = false;
    return true;
}

bool WorhpInterface::solve()
{
    if (!m_problem) {
        reportError("WorhpInterface", "solve", "Optimization problem not set.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    if (!m_problem->prepare()){
        reportError("WorhpInterface", "solve", "Error while preparing the optimization problem.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    unsigned int n = m_problem->numberOfVariables();
    unsigned int m = m_problem->numberOfConstraints();
    unsigned int nonZerosJacobian = 0;
    unsigned int nonZerosHessian = 0;

    if (m_problem->info().hasSparseConstraintJacobian()) {
        if (!(m_problem->getConstraintsJacobianInfo(m_pimpl->constraintsJacNNZRows,
                                                    m_pimpl->constraintsJacNNZCols))){
            reportError("WorhpInterface", "solve", "Error while retrieving constraints jacobian info.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        if (m_pimpl->constraintsJacNNZCols.size() != m_pimpl->constraintsJacNNZRows.size()) {
            reportError("WorhpInterface", "solve", "The vectors of nonzeros of the constraints jacobian have different sizes.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        nonZerosJacobian = static_cast<unsigned int>(m_pimpl->constraintsJacNNZRows.size());

        m_pimpl->orderedJacobianNonZeros.resize(m_pimpl->constraintsJacNNZRows.size());

        for (size_t i = 0; i < m_pimpl->constraintsJacNNZRows.size(); ++i) {
            m_pimpl->orderedJacobianNonZeros[i].row = static_cast<unsigned int>(m_pimpl->constraintsJacNNZRows[i]);
            m_pimpl->orderedJacobianNonZeros[i].col = static_cast<unsigned int>(m_pimpl->constraintsJacNNZCols[i]);
        }

        std::sort(m_pimpl->orderedJacobianNonZeros.begin(), m_pimpl->orderedJacobianNonZeros.end());

        m_pimpl->sparseJacobian = true;

    } else {
        nonZerosJacobian = m*n;
        m_pimpl->sparseJacobian = false;
    }

    if (!(m_problem->info().hessianIsProvided()) || (m_pimpl->useApproximatedHessian)) {
        if (!setWorhpParam("UserHM", false)) {
            reportError("WorhpInterface", "solve", "Failed to set UserHM option.");
            m_pimpl->previouslySolved = false;
            return false;
        }
    }

    if (m_problem->info().hasSparseHessian()) {
        if (!(m_problem->getHessianInfo(m_pimpl->hessianNNZRows,
                                        m_pimpl->hessianNNZCols))){
            reportError("WorhpInterface", "solve", "Error while retrieving hessian info.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        if (m_pimpl->hessianNNZRows.size() != m_pimpl->hessianNNZCols.size()) {
            reportError("WorhpInterface", "solve", "The vectors of nonzeros of the hessian matrix have different sizes.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        m_pimpl->hessianLowerTriangularNonZeros = 0;
        for (size_t i = 0; i < m_pimpl->hessianNNZRows.size(); ++i) {
            if (m_pimpl->hessianNNZRows[i] > m_pimpl->hessianNNZCols[i]) {
                m_pimpl->hessianLowerTriangularNonZeros++;
            }
        }

        nonZerosHessian = static_cast<unsigned int>(m_pimpl->hessianLowerTriangularNonZeros) + n; //add the diagonal elements

        m_pimpl->orderedHessianNonZeros.resize(m_pimpl->hessianNNZRows.size());

        for (size_t i = 0; i < m_pimpl->hessianNNZRows.size(); ++i) {
            m_pimpl->orderedHessianNonZeros[i].row = static_cast<unsigned int>(m_pimpl->hessianNNZRows[i]);
            m_pimpl->orderedHessianNonZeros[i].col = static_cast<unsigned int>(m_pimpl->hessianNNZCols[i]);
        }

        std::sort(m_pimpl->orderedHessianNonZeros.begin(), m_pimpl->orderedHessianNonZeros.end(),
                  [](const MatrixElement &a, const MatrixElement &b) {return a.lowerTriangularCompare(b);});

        m_pimpl->sparseHessian = true;

    } else {
        nonZerosHessian = n*(n + 1)/2;
        if (!setWorhpParam("UserHMstructure", 0)) {
            reportError("WorhpInterface", "solve", "Failed to set UserHMstructure option.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        if (!setWorhpParam("FidifHM", true)) {
            reportError("WorhpInterface", "solve", "Failed to set BFGSmethod option.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        m_pimpl->sparseHessian = false;
    }

    if (m_pimpl->possibleReOptimize(n, m, nonZerosJacobian, nonZerosHessian)) {

        WorhpRestart (&(m_pimpl->worhp.opt), &(m_pimpl->worhp.wsp), &(m_pimpl->worhp.par), &(m_pimpl->worhp.cnt));

    } else {

        if (m_pimpl->firstTime) {
            m_pimpl->firstTime = false;
        } else {
            WorhpFree(&(m_pimpl->worhp.opt), &(m_pimpl->worhp.wsp), &(m_pimpl->worhp.par), &(m_pimpl->worhp.cnt));

            WorhpPreInit(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);

            InitParams(&(m_pimpl->worhp.status), &(m_pimpl->worhp.par));

            for (auto boolParam = m_pimpl->boolParamsBackup.begin(); boolParam != m_pimpl->boolParamsBackup.end(); ++boolParam) {
                WorhpSetBoolParam(&(m_pimpl->worhp.par), boolParam->first.c_str() , boolParam->second);
            }

            for (auto intParam = m_pimpl->intParamsBackup.begin(); intParam != m_pimpl->intParamsBackup.end(); ++intParam) {
                WorhpSetIntParam(&(m_pimpl->worhp.par), intParam->first.c_str() , intParam->second);
            }

            for (auto doubleParam = m_pimpl->doubleParamsBackup.begin(); doubleParam != m_pimpl->doubleParamsBackup.end(); ++doubleParam) {
                WorhpSetDoubleParam(&(m_pimpl->worhp.par), doubleParam->first.c_str() , doubleParam->second);
            }
        }

        m_pimpl->worhp.opt.n = static_cast<int>(n);
        m_pimpl->worhp.opt.m = static_cast<int>(m);

        m_pimpl->worhp.wsp.DF.nnz = static_cast<int>(n); //Assume dense gradient

        m_pimpl->worhp.wsp.DG.nnz = static_cast<int>(nonZerosJacobian);

        m_pimpl->worhp.wsp.HM.nnz = static_cast<int>(nonZerosHessian);

        m_pimpl->resizeBuffers(n, m);

        // plus full diagonal
        WorhpInit(&(m_pimpl->worhp.opt), &(m_pimpl->worhp.wsp), &(m_pimpl->worhp.par), &(m_pimpl->worhp.cnt));

        if (m_pimpl->worhp.cnt.status != FirstCall)
        {
            reportError("WorhpInterface", "solve", "Failed to initialize WORHP structures.");
            m_pimpl->previouslySolved = false;
            return false;
        }

        if (m_problem->info().costIsLinear()) {
            m_pimpl->worhp.opt.FType = WORHP_LINEAR;
        } else if (m_problem->info().costIsQuadratic()) {
            m_pimpl->worhp.opt.FType = WORHP_QUADRATIC;
        } else {
            m_pimpl->worhp.opt.FType = WORHP_NONLINEAR;
        }

        if (!(m_problem->info().hasNonLinearConstraints())) {
            Eigen::Map<Eigen::VectorXi> gTypesMap(m_pimpl->worhp.opt.GType, m_pimpl->worhp.opt.m);
            gTypesMap.setConstant(WORHP_LINEAR);
        } else {
            Eigen::Map<Eigen::VectorXi> gTypesMap(m_pimpl->worhp.opt.GType, m_pimpl->worhp.opt.m);
            gTypesMap.setConstant(WORHP_NONLINEAR);
        }


        Eigen::Map<Eigen::VectorXd> initialBoundsMultipliersMap(m_pimpl->worhp.opt.Lambda, n);
        initialBoundsMultipliersMap.setZero();

        Eigen::Map<Eigen::VectorXd> initialConstraintsMultipliersMap(m_pimpl->worhp.opt.Mu, m);
        initialConstraintsMultipliersMap.setZero();

        Eigen::Map<Eigen::VectorXd> initialPointMap(m_pimpl->worhp.opt.X, n);
        initialPointMap.setZero();
    }

    if (m_problem->getGuess(m_pimpl->initialPoint)) {
        if (m_pimpl->initialPoint.size() != n) {
            reportError("WorhpInterface", "solve", "The specified guess size does not match the number of variables.");
            m_pimpl->previouslySolved = false;
            return false;
        }
        Eigen::Map<Eigen::VectorXd> initialPointMap(m_pimpl->worhp.opt.X, n);
        initialPointMap = iDynTree::toEigen(m_pimpl->initialPoint);
    }

    if (m_problem->getVariablesLowerBound(m_pimpl->variablesLowerBounds)) {
        if (m_pimpl->variablesLowerBounds.size() != n) {
            reportError("WorhpInterface", "solve", "The size of the variables lowerbound does not match the number of variables.");
            m_pimpl->previouslySolved = false;
            return false;
        }
        Eigen::Map<Eigen::VectorXd> variablesLowerBoundMap(m_pimpl->worhp.opt.XL, n);
        variablesLowerBoundMap = iDynTree::toEigen(m_pimpl->variablesLowerBounds);
    } else {
        Eigen::Map<Eigen::VectorXd> variablesLowerBoundMap(m_pimpl->worhp.opt.XL, n);
        variablesLowerBoundMap.setConstant(-m_pimpl->worhp.par.Infty);
    }

    if (m_problem->getVariablesUpperBound(m_pimpl->variablesUpperBounds)) {
        if (m_pimpl->variablesUpperBounds.size() != n) {
            reportError("WorhpInterface", "solve", "The size of the variables upperbound does not match the number of variables.");
            m_pimpl->previouslySolved = false;
            return false;
        }
        Eigen::Map<Eigen::VectorXd> variablesUpperBoundMap(m_pimpl->worhp.opt.XU, n);
        variablesUpperBoundMap = iDynTree::toEigen(m_pimpl->variablesUpperBounds);
    } else {
        Eigen::Map<Eigen::VectorXd> variablesUpperBoundMap(m_pimpl->worhp.opt.XU, n);
        variablesUpperBoundMap.setConstant(m_pimpl->worhp.par.Infty);
    }

    if (!(m_problem->getConstraintsBounds(m_pimpl->constraintsLowerBounds, m_pimpl->constraintsUpperBounds))) {
        reportError("WorhpInterface", "solve", "Failed to get constraints bounds.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    if (m_pimpl->constraintsLowerBounds.size() != m_pimpl->constraintsUpperBounds.size()) {
        reportError("WorhpInterface", "solve", "Constraints bounds have different sizes.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    if (m_pimpl->constraintsLowerBounds.size() != m) {
        reportError("WorhpInterface", "solve", "Constraints bounds have size different from the number of constraints.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    Eigen::Map<Eigen::VectorXd> constraintsLowerBoundMap(m_pimpl->worhp.opt.GL, m);
    constraintsLowerBoundMap = iDynTree::toEigen(m_pimpl->constraintsLowerBounds);

    Eigen::Map<Eigen::VectorXd> constraintsUpperBoundMap(m_pimpl->worhp.opt.GU, m);
    constraintsUpperBoundMap = iDynTree::toEigen(m_pimpl->constraintsUpperBounds);

    /*
    * Specify matrix structures in CS format, using Fortran indexing,
    * i.e. 1...N instead of 0...N-1, to describe the matrix structure.
    */
    // Define DF as row vector, column index is ommited
    if (m_pimpl->worhp.wsp.DF.NeedStructure) {
        for (int i = 0; i < static_cast<int>(n); ++i) {
            m_pimpl->worhp.wsp.DF.row[i] = i + 1; //dense
        }
    }
    // Define DG as CS-matrix
    if (m_pimpl->worhp.wsp.DG.NeedStructure) {
        // only set the nonzero entries in column-major order
        if (m_pimpl->sparseJacobian) {
            for (size_t el = 0; el < m_pimpl->orderedJacobianNonZeros.size(); ++el) {
                m_pimpl->worhp.wsp.DG.row[el] = static_cast<int>(m_pimpl->orderedJacobianNonZeros[el].row) + 1;
                m_pimpl->worhp.wsp.DG.col[el] = static_cast<int>(m_pimpl->orderedJacobianNonZeros[el].col) + 1;
            }
        } else {
            size_t element = 0;
            for (unsigned int  col = 0; col < n; ++col) {
                for (unsigned int row = 0; row < m; ++row) {
                    m_pimpl->worhp.wsp.DG.row[element] = static_cast<int>(row) + 1;
                    m_pimpl->worhp.wsp.DG.col[element] = static_cast<int>(col) + 1;
                    element++;
                }
            }
        }
    }

    if (m_pimpl->worhp.wsp.HM.NeedStructure) {
        /*
        * only set the nonzero entries, with the strictly lower triangle first,
        * followed by ALL diagonal entries
        */
        // strict lower triangle
        if (m_pimpl->sparseHessian) {
            for (size_t el = 0; el < m_pimpl->hessianLowerTriangularNonZeros; ++el) {
                m_pimpl->worhp.wsp.HM.row[el] = static_cast<int>(m_pimpl->orderedHessianNonZeros[el].row) + 1;
                m_pimpl->worhp.wsp.HM.col[el] = static_cast<int>(m_pimpl->orderedHessianNonZeros[el].col) + 1;
            }

        } else {
            size_t element = 0;
            for (unsigned int  col = 1; col < n; ++col) {
                for (unsigned int row = 0; row < col; ++row) {
                    m_pimpl->worhp.wsp.HM.row[element] = static_cast<int>(row) + 1;
                    m_pimpl->worhp.wsp.HM.col[element] = static_cast<int>(col) + 1;
                    element++;
                }
            }
        }
        // diagonal
        for (int i = 0; i < m_pimpl->worhp.opt.n; i++) {
            m_pimpl->worhp.wsp.HM.row[m_pimpl->worhp.wsp.HM.nnz - m_pimpl->worhp.opt.n + i] = i + 1;
            m_pimpl->worhp.wsp.HM.col[m_pimpl->worhp.wsp.HM.nnz - m_pimpl->worhp.opt.n + i] = i + 1;
        }
    }

    Eigen::Map<const Eigen::VectorXd> variablesMap(m_pimpl->worhp.opt.X, m_pimpl->worhp.opt.n);
    iDynTree::toEigen(m_pimpl->variablesBuffer) = variablesMap;
    if (!m_problem->setVariables(m_pimpl->variablesBuffer)) {
        reportError("WorhpInterface", "solve", "Failed to update variables.");
        m_pimpl->previouslySolved = false;
        return false;
    }

    /*
    * WORHP Reverse Communication loop.
    * In every iteration poll GetUserAction for the requested action, i.e. one
    * of {callWorhp, iterOutput, evalF, evalG, evalDF, evalDG, evalHM, fidif}.
    *
    * Make sure to reset the requested user action afterwards by calling
    * DoneUserAction, except for 'callWorhp' and 'fidif'.
    */
    while (m_pimpl->worhp.cnt.status < TerminateSuccess && m_pimpl->worhp.cnt.status > TerminateError) {
        /*
        * WORHP's main routine.
        * Do not manually reset callWorhp, this is only done by the FD routines.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, callWorhp)) {
            Worhp(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
            // No DoneUserAction!
        }

        /*
        * Show iteration output.
        * The call to IterationOutput() may be replaced by user-defined code.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, iterOutput)) {
            IterationOutput(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
            DoneUserAction(&m_pimpl->worhp.cnt, iterOutput);
        }

        /*
        * Evaluate the objective function.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, evalF)) {

            if (!(m_pimpl->updateVariables(m_problem))) {
                    return false;
            }

            double costValue = 0;
            if (!m_problem->evaluateCostFunction(costValue)) {
                reportError("WorhpInterface", "solve", "Failed to evaluate cost.");
                m_pimpl->previouslySolved = false;
                return false;
            }

            m_pimpl->worhp.opt.F = m_pimpl->worhp.wsp.ScaleObj * costValue;

            DoneUserAction(&m_pimpl->worhp.cnt, evalF);
        }

        /*
        * Evaluate the constraints.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, evalG)) {

            if (!(m_pimpl->updateVariables(m_problem))) {
                    return false;
            }

            if (!m_problem->evaluateConstraints(m_pimpl->constraintsEvaluationBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to evaluate constraints.");
                m_pimpl->previouslySolved = false;
                return false;
            }
            assert(m_pimpl->constraintsEvaluationBuffer.size() == m);
            Eigen::Map<Eigen::VectorXd> constraintsMap(m_pimpl->worhp.opt.G, m);
            constraintsMap = iDynTree::toEigen(m_pimpl->constraintsEvaluationBuffer);

            DoneUserAction(&m_pimpl->worhp.cnt, evalG);
        }

        /*
        * Evaluate the gradient of the objective function.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, evalDF)) {

            if (!(m_pimpl->updateVariables(m_problem))) {
                    return false;
            }

            if (!m_problem->evaluateCostGradient(m_pimpl->costGradientBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to evaluate cost gradient.");
                m_pimpl->previouslySolved = false;
                return false;
            }
            assert(m_pimpl->costGradientBuffer.size() == n);
            Eigen::Map<Eigen::VectorXd> costGradientMap(m_pimpl->worhp.wsp.DF.val, n); //dense vector
            costGradientMap = m_pimpl->worhp.wsp.ScaleObj *  iDynTree::toEigen(m_pimpl->costGradientBuffer);
            DoneUserAction(&m_pimpl->worhp.cnt, evalDF);
        }

        /*
        * Evaluate the Jacobian of the constraints.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, evalDG)) {

            if (!(m_pimpl->updateVariables(m_problem))) {
                    return false;
            }

            if (!m_problem->evaluateConstraintsJacobian(m_pimpl->constraintsJacobianBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to evaluate constraints jacobian.");
                m_pimpl->previouslySolved = false;
                return false;
            }

            if (m_pimpl->sparseJacobian) {
                for (size_t i = 0; i < m_pimpl->orderedJacobianNonZeros.size(); ++i) {
                    m_pimpl->worhp.wsp.DG.val[i] =
                            m_pimpl->constraintsJacobianBuffer(
                                m_pimpl->orderedJacobianNonZeros[i].row,
                                m_pimpl->orderedJacobianNonZeros[i].col);
                }
            } else {
                size_t element = 0;
                for (unsigned int  col = 0; col < n; ++col) {
                    for (unsigned int row = 0; row < m; ++row) {
                        m_pimpl->worhp.wsp.DG.val[element] = m_pimpl->constraintsJacobianBuffer(row, col);
                        element++;
                    }
                }
            }

            DoneUserAction(&m_pimpl->worhp.cnt, evalDG);
        }

        /*
        * Evaluate the Hessian matrix of the Lagrange function (L = f + mu*g)
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, evalHM)) {

            if (!(m_pimpl->updateVariables(m_problem))) {
                    return false;
            }

            if (!m_problem->evaluateCostHessian(m_pimpl->costHessianBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to get cost hessian.");
                m_pimpl->previouslySolved = false;
                return false;
            }

            Eigen::Map<Eigen::VectorXd> constraintsMultipliersMap(m_pimpl->worhp.opt.Mu, m);
            iDynTree::toEigen(m_pimpl->constraintsMultipliersBuffer) = constraintsMultipliersMap;

            if (!m_problem->evaluateConstraintsHessian(m_pimpl->constraintsMultipliersBuffer, m_pimpl->constraintsHessianBuffer)) {
                reportError("WorhpInterface", "solve", "Failed to get constraints hessian.");
                m_pimpl->previouslySolved = false;
                return false;
            }

            if (m_pimpl->sparseHessian) {
                for (size_t i = 0; i < m_pimpl->hessianLowerTriangularNonZeros; ++i) {
                    unsigned int row = m_pimpl->orderedJacobianNonZeros[i].row;
                    unsigned int col = m_pimpl->orderedJacobianNonZeros[i].col;
                    m_pimpl->worhp.wsp.HM.val[i] =
                            (m_pimpl->worhp.wsp.ScaleObj * m_pimpl->costHessianBuffer(row, col)) +
                            m_pimpl->constraintsHessianBuffer(row,col);
                }
            } else {
                size_t element = 0;
                for (unsigned int  col = 1; col < n; ++col) {
                    for (unsigned int row = 0; row < col; ++row) {
                        m_pimpl->worhp.wsp.HM.val[element] =
                                (m_pimpl->worhp.wsp.ScaleObj * m_pimpl->costHessianBuffer(row, col)) +
                                m_pimpl->constraintsHessianBuffer(row,col);
                        element++;
                    }
                }
            }
            // diagonal
            for (unsigned int i = 0; i < n; i++) {
                m_pimpl->worhp.wsp.HM.val[i] =
                        (m_pimpl->worhp.wsp.ScaleObj * m_pimpl->costHessianBuffer(i, i)) +
                        m_pimpl->constraintsHessianBuffer(i,i);
            }

            DoneUserAction(&m_pimpl->worhp.cnt, evalHM);
        }

        /*
        * Use finite differences with RC to determine derivatives
        * Do not reset fidif, this is done by the FD routine.
        */
        if (GetUserAction(&m_pimpl->worhp.cnt, fidif)) {
            WorhpFidif(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
            // No DoneUserAction!
        }
    }

    if (m_pimpl->worhp.cnt.status <= TerminateError) {
        reportError("WorhpInterface", "solve", "Failed to solve the problem. Here additional infos:");
        m_pimpl->previouslySolved = false;
        int printLevel;
        getWorhpParam("NLPprint", printLevel);
        setWorhpParam("NLPprint", 0);
        StatusMsg(&m_pimpl->worhp.opt, &m_pimpl->worhp.wsp, &m_pimpl->worhp.par, &m_pimpl->worhp.cnt);
        setWorhpParam("NLPprint", printLevel);
        return false;
    }
    m_pimpl->previouslySolved = true;
    m_pimpl->previousNumberOfVariables = static_cast<unsigned int>(m_pimpl->worhp.opt.n);
    m_pimpl->previousNumberOfConstraints = static_cast<unsigned int>(m_pimpl->worhp.opt.m);
    m_pimpl->previousJacobianNonZeros = static_cast<unsigned int>(m_pimpl->worhp.wsp.DG.nnz);
    m_pimpl->previousHessianNonZeros = static_cast<unsigned int>(m_pimpl->worhp.wsp.HM.nnz);


    return true;
}

bool WorhpInterface::getPrimalVariables(iDynTree::VectorDynSize &primalVariables)
{
    if (!(m_pimpl->previouslySolved)) {
        reportError("WorhpInterface", "getPrimalVariables", "The solve method was not called or it returned false.");
        return false;
    }

    Eigen::Map<const Eigen::VectorXd> variablesMap(m_pimpl->worhp.opt.X, m_pimpl->worhp.opt.n);
    iDynTree::toEigen(m_pimpl->variablesBuffer) = variablesMap;

    primalVariables = m_pimpl->variablesBuffer;

    return true;
}

bool WorhpInterface::getDualVariables(iDynTree::VectorDynSize &constraintsMultipliers, iDynTree::VectorDynSize &lowerBoundsMultipliers, iDynTree::VectorDynSize &upperBoundsMultipliers)
{
    if (!(m_pimpl->previouslySolved)) {
        reportError("WorhpInterface", "getDualVariables", "The solve method was not called or it returned false.");
        return false;
    }

    constraintsMultipliers.resize(static_cast<unsigned int>(m_pimpl->previousNumberOfConstraints));
    lowerBoundsMultipliers.resize(static_cast<unsigned int>(m_pimpl->previousNumberOfVariables));
    upperBoundsMultipliers.resize(static_cast<unsigned int>(m_pimpl->previousNumberOfVariables));

    Eigen::Map<const Eigen::VectorXd> constraintsMultipliersMap(m_pimpl->worhp.opt.Mu, m_pimpl->worhp.opt.m);
    iDynTree::toEigen(constraintsMultipliers) = constraintsMultipliersMap;

    if (m_pimpl->variablesLowerBounds.size() == 0) {
        lowerBoundsMultipliers.zero();

        if (m_pimpl->variablesUpperBounds.size() == 0) {
            upperBoundsMultipliers.zero();
        }
        else {
            Eigen::Map<const Eigen::VectorXd> variablesMultipliersMap(m_pimpl->worhp.opt.Lambda, m_pimpl->worhp.opt.n);
            iDynTree::toEigen(upperBoundsMultipliers) = variablesMultipliersMap;
        }
    } else if (m_pimpl->variablesUpperBounds.size() == 0) {
        upperBoundsMultipliers.zero();
        Eigen::Map<const Eigen::VectorXd> variablesMultipliersMap(m_pimpl->worhp.opt.Lambda, m_pimpl->worhp.opt.n);
        iDynTree::toEigen(lowerBoundsMultipliers) = variablesMultipliersMap;
    } else {
        double lowerBoundDistance, upperBoundDistance;

        for (unsigned int i = 0;  i < lowerBoundsMultipliers.size(); ++i) {
            lowerBoundDistance = std::fabs(m_pimpl->worhp.opt.X[i] - m_pimpl->variablesLowerBounds(i));
            upperBoundDistance = std::fabs(m_pimpl->worhp.opt.X[i] - m_pimpl->variablesUpperBounds(i));

            if (lowerBoundDistance < upperBoundDistance) { //Lower bounds are active
                lowerBoundsMultipliers(i) = m_pimpl->worhp.opt.Lambda[i];
                upperBoundsMultipliers(i) = 0.0;
            } else {
                lowerBoundsMultipliers(i) = 0.0;
                upperBoundsMultipliers(i) = m_pimpl->worhp.opt.Lambda[i];
            }
        }
    }

    return true;
}

bool WorhpInterface::getOptimalCost(double &optimalCost)
{
    if (!(m_pimpl->previouslySolved)) {
        reportError("WorhpInterface", "getOptimalCost", "The solve method was not called or it returned false.");
        return false;
    }

    if (!iDynTree::checkDoublesAreEqual(m_pimpl->worhp.wsp.ScaleObj, 0.0)) {
        optimalCost = m_pimpl->worhp.opt.F / m_pimpl->worhp.wsp.ScaleObj;
    } else {
        optimalCost = m_pimpl->worhp.opt.F;
    }

    return true;
}

bool WorhpInterface::getOptimalConstraintsValues(iDynTree::VectorDynSize &constraintsValues)
{
    if (!(m_pimpl->previouslySolved)) {
        reportError("WorhpInterface", "getOptimalConstraintsValues", "The solve method was not called or it returned false.");
        return false;
    }

    Eigen::Map<const Eigen::VectorXd> constraintsMap(m_pimpl->worhp.opt.G, m_pimpl->worhp.opt.m);
    iDynTree::toEigen(m_pimpl->constraintsEvaluationBuffer) = constraintsMap;

    constraintsValues = m_pimpl->constraintsEvaluationBuffer;

    return true;
}

double WorhpInterface::minusInfinity()
{
    return -m_pimpl->worhp.par.Infty;
}

double WorhpInterface::plusInfinity()
{
    return m_pimpl->worhp.par.Infty;
}

void WorhpInterface::useApproximatedHessians(bool useApproximatedHessian)
{
    m_pimpl->useApproximatedHessian = useApproximatedHessian;
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, bool value)
{
    if (!WorhpSetBoolParam(&(m_pimpl->worhp.par) ,paramName.c_str() , value)) {
        reportError("WorhpInterface", "setWorhpParam", ("Failed to set parameter " + paramName + ". Either it was not found or the expected input value has wrong type.").c_str());
        return false;
    }
    m_pimpl->boolParamsBackup[paramName] = value;
    return true;
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, double value)
{
    if (!WorhpSetDoubleParam(&(m_pimpl->worhp.par) ,paramName.c_str() , value)) {
        reportError("WorhpInterface", "setWorhpParam", ("Failed to set parameter " + paramName + ". Either it was not found or the expected input value has wrong type.").c_str());
        return false;
    }
    m_pimpl->doubleParamsBackup[paramName] = value;
    return true;
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, int value)
{
    if (!WorhpSetIntParam(&(m_pimpl->worhp.par) ,paramName.c_str() , value)) {
        reportError("WorhpInterface", "setWorhpParam", ("Failed to set parameter " + paramName + ". Either it was not found or the expected input value has wrong type.").c_str());
        return false;
    }
    m_pimpl->intParamsBackup[paramName] = value;
    return true;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, bool &value)
{
    if (!WorhpGetBoolParam(&(m_pimpl->worhp.par) ,paramName.c_str() , &value)) {
        reportError("WorhpInterface", "getWorhpParam", ("Failed to get parameter " + paramName + ". Either it was not found or the expected output value has wrong type.").c_str());
        return false;
    }
    return true;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, double &value)
{
    if (!WorhpGetDoubleParam(&(m_pimpl->worhp.par) ,paramName.c_str() , &value)) {
        reportError("WorhpInterface", "getWorhpParam", ("Failed to get parameter " + paramName + ". Either it was not found or the expected output value has wrong type.").c_str());
        return false;
    }
    return true;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, int &value)
{
    if (!WorhpGetIntParam(&(m_pimpl->worhp.par) ,paramName.c_str() , &value)) {
        reportError("WorhpInterface", "getWorhpParam", ("Failed to get parameter " + paramName + ". Either it was not found or the expected output value has wrong type.").c_str());
        return false;
    }
    return true;
}
