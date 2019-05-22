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
#ifndef HAVE_STDDEF_H
    #ifndef HAVE_CSTDDEF
        #define HAVE_CSTDDEF
        #include <IpTNLP.hpp>
        #undef HAVE_CSTDDEF
    #else
        #include <IpTNLP.hpp>
    #endif
    #undef HAVE_STDDEF_H
#else
    #ifndef HAVE_CSTDDEF
        #define HAVE_CSTDDEF
        #include <IpTNLP.hpp>
        #undef HAVE_CSTDDEF
    #else
        #include <IpTNLP.hpp>
    #endif
#endif
#include <IpIpoptApplication.hpp>

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/Core/Utils.h>

#include <cassert>
#include <ostream>

namespace iDynTree {
    namespace optimization {

        class NLPImplementation : public Ipopt::TNLP {
            // IPOPT methods redefinition
            VectorDynSize m_constraintsLowerBounds, m_constraintsUpperBounds, m_variablesLowerBounds, m_variablesUpperBounds;
            VectorDynSize m_variablesBuffer, m_costGradientBuffer, m_constraintsBuffer;
            MatrixDynSize m_jacobianBuffer, m_costHessianBuffer, m_constraintsHessianBuffer, m_lagrangianHessianBuffer;
        public:

            unsigned int numberOfVariables, numberOfConstraints;
            std::vector<size_t> constraintsJacNNZRows, constraintsJacNNZCols, inputHessianNNZRows, inputHessianNNZCols,
                lowerTriangularHessianNNZRows, lowerTriangularHessianNNZCols;
            std::shared_ptr<OptimizationProblem> problem;
            double minusInfinity, plusInfinity; //TODO. Set these before solving
            VectorDynSize solution;
            VectorDynSize initialGuess;
            bool initialGuessSet; //TODO. Reset to false when finalizing the solution
            int exitCode;
            double optimalCostValue;
            VectorDynSize optimalConstraintsValue;
            VectorDynSize lowerBoundMultipliers, upperBoundMultipliers, constraintMultipliers;

            NLPImplementation()
            : minusInfinity(-1e19)
            , plusInfinity(1e19)
            , initialGuessSet(false)
            , exitCode(-6)
            {}

            virtual ~NLPImplementation() override;

            virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) override {

                n = static_cast<Ipopt::Index>(numberOfVariables); //set in the solve method

                if (m_variablesBuffer.size() != numberOfVariables) {
                    m_variablesBuffer.resize(numberOfVariables);
                }

                if (m_costGradientBuffer.size() != numberOfVariables) {
                    m_costGradientBuffer.resize(numberOfVariables);
                }

                if ((m_costHessianBuffer.rows() != numberOfVariables) || (m_costHessianBuffer.cols() != numberOfVariables)) {
                    m_costHessianBuffer.resize(numberOfVariables, numberOfVariables);
                }

                if ((m_constraintsHessianBuffer.rows() != numberOfVariables) || (m_constraintsHessianBuffer.cols() != numberOfVariables)) {
                    m_constraintsHessianBuffer.resize(numberOfVariables, numberOfVariables);
                }

                if ((m_lagrangianHessianBuffer.rows() != numberOfVariables) || (m_lagrangianHessianBuffer.cols() != numberOfVariables)) {
                    m_lagrangianHessianBuffer.resize(numberOfVariables, numberOfVariables);
                }

                m = static_cast<Ipopt::Index>(numberOfConstraints); //set in the solve method

                if (m_constraintsBuffer.size() != numberOfConstraints) {
                    m_constraintsBuffer.resize(numberOfConstraints);
                }

                if (constraintMultipliers.size() != numberOfConstraints) {
                    constraintMultipliers.resize(numberOfConstraints);
                    constraintMultipliers.zero();
                }

                if ((m_jacobianBuffer.rows() != numberOfConstraints) || (m_jacobianBuffer.cols() != numberOfVariables)) {
                    m_jacobianBuffer.resize(numberOfConstraints, numberOfVariables);
                }

                nnz_jac_g = static_cast<Ipopt::Index>(constraintsJacNNZRows.size()); //set in the solve method

                nnz_h_lag = static_cast<Ipopt::Index>(lowerTriangularHessianNNZRows.size());

                index_style = C_STYLE;
                return true;
            }


            virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) override {

                bool variablesLowerBounded = false, variablesUpperBounded = false;
                if (problem->getVariablesLowerBound(m_variablesLowerBounds)){
                    variablesLowerBounded = true;
                }
                if (problem->getVariablesUpperBound(m_variablesUpperBounds)){
                    variablesUpperBounded = true;
                }

                Eigen::Map<Eigen::VectorXd> variablesLB(x_l, n);
                Eigen::Map<Eigen::VectorXd> variablesUB(x_u, n);
                Eigen::Map<Eigen::VectorXd> variablesLBInput = toEigen(m_variablesLowerBounds);
                Eigen::Map<Eigen::VectorXd> variablesUBInput = toEigen(m_variablesUpperBounds);

                if (variablesLowerBounded) {
                    variablesLB = variablesLBInput;
                } else {
                    variablesLB.setConstant(minusInfinity);
                }

                if (variablesUpperBounded) {
                    variablesUB = variablesUBInput;
                } else {
                    variablesUB.setConstant(plusInfinity);
                }

                if (!(problem->getConstraintsBounds(m_constraintsLowerBounds, m_constraintsUpperBounds))){
                    reportError("IpoptInterface", "solve", "Error while retrieving constraints info.");
                    return false;
                }
                Eigen::Map<Eigen::VectorXd> constraintsLowerBoundsMap(g_l, m);
                Eigen::Map<Eigen::VectorXd> constraintsUpperBoundsMap(g_u, m);

                constraintsLowerBoundsMap = toEigen(m_constraintsLowerBounds);
                constraintsUpperBoundsMap = toEigen(m_constraintsUpperBounds);

                return true;
            }

            virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda) override {
                if (init_z) {
                    Eigen::Map<Eigen::VectorXd> zlMap(z_L, n);
                    Eigen::Map<Eigen::VectorXd> zuMap(z_U, n);
                    if ((lowerBoundMultipliers.size() == static_cast<unsigned int>(n)) &&  (upperBoundMultipliers.size() == static_cast<unsigned int>(n))){
                        zlMap = iDynTree::toEigen(lowerBoundMultipliers);
                        zuMap = iDynTree::toEigen(upperBoundMultipliers);
                    } else {
                        zlMap.setZero();
                        zuMap.setZero();
                    }
                }
                if (init_lambda)
                {
                    Eigen::Map<Eigen::VectorXd> lambdaMap(lambda, m);
                    if (constraintMultipliers.size() == static_cast<unsigned int>(m)) {
                        lambdaMap = iDynTree::toEigen(constraintMultipliers);
                    } else {
                        lambdaMap.setZero();
                    }
                }

                if(init_x){
                    Eigen::Map<Eigen::VectorXd> x_map(x, n);
                    initialGuessSet = problem->getGuess(initialGuess);
                    if (initialGuessSet) {
                        if (initialGuess.size() == static_cast<unsigned int>(n)){
                            x_map = iDynTree::toEigen(initialGuess);
                        } else {
                            reportWarning("NLPImplementation", "get_starting_point", "The specified initial guess has dimension different from the number of variables. Ignoring.");
                            if (solution.size() == static_cast<unsigned int>(n)) {
                                x_map = toEigen(solution);
                            } else {
                                x_map.setZero();
                            }
                        }
                    } else {
                        if (solution.size() == static_cast<unsigned int>(n)) {
                            x_map = toEigen(solution);
                        } else {
                            x_map.setZero();
                        }
                    }
                }
                return true;
            }

            virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value) override {

                if (new_x){
                    Eigen::Map<const Eigen::VectorXd> x_map(x, n);
                    toEigen(m_variablesBuffer) = x_map;
                    if (!(problem->setVariables(m_variablesBuffer))){
                        reportError("NLPImplementation", "eval_f", "Error while setting the optimization variables.");
                        return false;
                    }
                }

                double costValue;

                if (!(problem->evaluateCostFunction(costValue))){
                    reportError("NLPImplementation", "eval_f", "Error while evaluating the cost function.");
                    return false;
                }

                obj_value = costValue;

                return true;
            }

            virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f) override {

                if (new_x){
                    Eigen::Map<const Eigen::VectorXd> x_map(x, n);
                    toEigen(m_variablesBuffer) = x_map;
                    if (!(problem->setVariables(m_variablesBuffer))){
                        reportError("NLPImplementation", "eval_grad_f", "Error while setting the optimization variables.");
                        return false;
                    }
                }

                if (!(problem->evaluateCostGradient(m_costGradientBuffer))){
                    reportError("NLPImplementation", "eval_grad_f", "Error while evaluating the cost function gradient.");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> gradientMap(grad_f, n);
                gradientMap = toEigen(m_costGradientBuffer);

                return true;
            }

            virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g) override {

                if (new_x){
                    Eigen::Map<const Eigen::VectorXd> x_map(x, n);
                    toEigen(m_variablesBuffer) = x_map;
                    if (!(problem->setVariables(m_variablesBuffer))){
                        reportError("NLPImplementation", "eval_g", "Error while setting the optimization variables.");
                        return false;
                    }
                }

                if (!(problem->evaluateConstraints(m_constraintsBuffer))){
                    reportError("NLPImplementation", "eval_g", "Error while evaluating the constraints.");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> constraintsMap(g, m);
                constraintsMap = toEigen(m_constraintsBuffer);

                return true;
            }

            virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values) override {

                if (new_x){
                    Eigen::Map<const Eigen::VectorXd> x_map(x, n);
                    toEigen(m_variablesBuffer) = x_map;
                    if (!(problem->setVariables(m_variablesBuffer))){
                        reportError("NLPImplementation", "eval_jac_g", "Error while setting the optimization variables.");
                        return false;
                    }
                }

                if (values != nullptr){
                    if (!(problem->evaluateConstraintsJacobian(m_jacobianBuffer))){
                        reportError("NLPImplementation", "eval_jac_g", "Error while evaluating the constraints jacobian.");
                        return false;
                    }
                }

                for (size_t i = 0; i < constraintsJacNNZRows.size(); ++i){
                    if (values == nullptr){
                        iRow[i] = static_cast<Ipopt::Index>(constraintsJacNNZRows[i]); //these two vectors have been filled in the get_nlp_info method
                        jCol[i] = static_cast<Ipopt::Index>(constraintsJacNNZCols[i]);
                    } else {
                        values[i] = m_jacobianBuffer(static_cast<unsigned int>(constraintsJacNNZRows[i]), static_cast<unsigned int>(constraintsJacNNZCols[i]));
                    }
                }
                return true;
            }

            virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values) override {

                if (new_x){
                    Eigen::Map<const Eigen::VectorXd> x_map(x, n);
                    toEigen(m_variablesBuffer) = x_map;
                    if (!(problem->setVariables(m_variablesBuffer))){
                        reportError("NLPImplementation", "eval_h", "Error while setting the optimization variables.");
                        return false;
                    }
                }

                if (values != nullptr){
                    if (!problem->evaluateCostHessian(m_costHessianBuffer)){
                        reportError("NLPImplementation", "eval_h", "Error while evaluating the cost hessian.");
                        return false;
                    }

                    if (new_x || new_lambda){
                        Eigen::Map<const Eigen::VectorXd> lambdaMap(lambda, m);
                        toEigen(constraintMultipliers) = lambdaMap;
                        if (!problem->evaluateConstraintsHessian(constraintMultipliers, m_constraintsHessianBuffer)){
                            reportError("NLPImplementation", "eval_h", "Error while evaluating the constraints hessian.");
                            return false;
                        }
                    }
                    toEigen(m_lagrangianHessianBuffer) = obj_factor * toEigen(m_costHessianBuffer) + toEigen(m_constraintsHessianBuffer);
                }

                for (size_t i = 0; i < lowerTriangularHessianNNZRows.size(); ++i){
                    if (values == nullptr){
                        iRow[i] = static_cast<Ipopt::Index>(lowerTriangularHessianNNZRows[i]); //these two vectors have been filled in the get_nlp_info method
                        jCol[i] = static_cast<Ipopt::Index>(lowerTriangularHessianNNZCols[i]);
                    } else {
                        values[i] = m_lagrangianHessianBuffer(static_cast<unsigned int>(lowerTriangularHessianNNZRows[i]),
                                                              static_cast<unsigned int>(lowerTriangularHessianNNZCols[i]));
                    }
                }

                return true;
            }

            virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq) override {
                if((status == Ipopt::SUCCESS)||status == Ipopt::STOP_AT_ACCEPTABLE_POINT){
                    Eigen::Map< const Eigen::VectorXd > x_map (x, n);

                    if (solution.size() != static_cast<unsigned int>(n)) {
                        solution.resize(static_cast<unsigned int>(n));
                    }

                    if (initialGuess.size() != static_cast<unsigned int>(n)) {
                        initialGuess.resize(static_cast<unsigned int>(n));
                    }

                    toEigen(solution) = x_map;
                    toEigen(initialGuess) = x_map;
                    initialGuessSet = false;

                    Eigen::Map<const Eigen::VectorXd> zlMap(z_L, n);
                    Eigen::Map<const Eigen::VectorXd> zuMap(z_U, n);

                    if (lowerBoundMultipliers.size() != static_cast<unsigned int>(n)) {
                        lowerBoundMultipliers.resize(static_cast<unsigned int>(n));
                    }

                    if (upperBoundMultipliers.size() != static_cast<unsigned int>(n)) {
                        upperBoundMultipliers.resize(static_cast<unsigned int>(n));
                    }

                    toEigen(lowerBoundMultipliers) = zlMap;
                    toEigen(upperBoundMultipliers) = zuMap;

                    Eigen::Map<const Eigen::VectorXd> lambdaMap(lambda, m);
                    toEigen(constraintMultipliers) = lambdaMap;

                    optimalCostValue = obj_value;

                    if (optimalConstraintsValue.size() != static_cast<unsigned int>(m)) {
                        optimalConstraintsValue.resize(static_cast<unsigned int>(m));
                    }

                    Eigen::Map<const Eigen::VectorXd> gMap(g, m);
                    toEigen(optimalConstraintsValue) = gMap;

                    if(status == Ipopt::SUCCESS){
                        exitCode = 0;
                    } else {
                        exitCode = 1;
                    }
                } else {
                    std::ostringstream errorMsg;
                    errorMsg << "Optimization problem failed due to ";
                    switch(status){

                        case Ipopt::LOCAL_INFEASIBILITY:
                            errorMsg << "local infeasibility!";
                            exitCode = -1;
                            break;

                        case Ipopt::MAXITER_EXCEEDED:
                        case Ipopt::CPUTIME_EXCEEDED:
                        case Ipopt::STOP_AT_TINY_STEP: //PREMATURE STOP
                            errorMsg << "a premature stop (max CPU time, max iterations or tiny steps)!";
                            exitCode = -2;
                            break;

                        case Ipopt::INVALID_NUMBER_DETECTED: //WRONG DATA INSERTION
                            errorMsg << "the detection of an invalid number!";
                            exitCode = -3;
                            break;

                        case Ipopt::DIVERGING_ITERATES:
                        case Ipopt::INTERNAL_ERROR:
                        case Ipopt::ERROR_IN_STEP_COMPUTATION:
                        case Ipopt::RESTORATION_FAILURE: //IPOPT INTERNAL PROBLEM
                            errorMsg << "an IPOPT internal problem (diverging iterates, error in step computation or restoration failure)!";
                            exitCode = -4;
                            break;

                        case Ipopt::USER_REQUESTED_STOP: //USER REQUESTED STOP
                            errorMsg << "an user requested stop!";
                            exitCode = -5;
                            break;

                        default:
                            errorMsg << "an unrecognized error!";
                            exitCode = -6;
                    }
                    reportError("IpoptInterface", "solve", errorMsg.str().c_str());
                }
            }
        };
        NLPImplementation::~NLPImplementation(){}


        class IpoptInterface::IpoptInterfaceImplementation {
        public:
            Ipopt::SmartPtr<NLPImplementation> nlpPointer;
            Ipopt::SmartPtr<Ipopt::IpoptApplication> loader;
            unsigned int previousNumberOfVariables, previousNumberOfConstraints;
            size_t previousJacobianNonZeros, previousHessianNonZeros;
            bool useApproximatedHessian;

            IpoptInterfaceImplementation()
            : nlpPointer(new NLPImplementation())
            , loader(IpoptApplicationFactory())
            , previousNumberOfVariables(0)
            , previousNumberOfConstraints(0)
            , previousJacobianNonZeros(0)
            , previousHessianNonZeros(0)
            , useApproximatedHessian(false)
            {
            }

            bool possibleReOptimize(){
                bool alreadySolved = (nlpPointer->exitCode >= 0);
                bool sameVariables = (previousNumberOfVariables == nlpPointer->numberOfVariables);
                bool sameConstraints = (previousNumberOfConstraints == nlpPointer->numberOfConstraints);
                bool sameNNZJac = (previousJacobianNonZeros == nlpPointer->constraintsJacNNZRows.size());
                bool sameNNZHes = (previousHessianNonZeros == nlpPointer->lowerTriangularHessianNNZRows.size());
                return alreadySolved && sameVariables && sameConstraints && sameNNZJac && sameNNZHes;
            }
        };

        IpoptInterface::IpoptInterface()
        : m_pimpl(new IpoptInterfaceImplementation())
        {
            assert(m_pimpl);
        }

        IpoptInterface::~IpoptInterface()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool IpoptInterface::isAvailable() const
        {
            return true;
        }

        bool IpoptInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
        {
            if (!problem){
                reportError("IpoptInterface", "setProblem", "Empty problem pointer.");
                return false;
            }
            m_problem = problem;
            m_pimpl->nlpPointer->problem = problem;
            return true;
        }

        bool IpoptInterface::solve()
        {
            if (!m_problem) {
                reportError("IpoptInterface", "solve", "Optimization problem not set.");
                return false;
            }

            m_pimpl->nlpPointer->minusInfinity = minusInfinity();
            m_pimpl->nlpPointer->plusInfinity = plusInfinity();

            if (!m_problem->prepare()){
                reportError("IpoptInterface", "solve", "Error while preparing the optimization problem.");
                return false;
            }

            m_pimpl->nlpPointer->numberOfVariables = m_problem->numberOfVariables();

            m_pimpl->nlpPointer->numberOfConstraints = m_problem->numberOfConstraints();

            if (!(m_problem->info().hessianIsProvided()) || (m_pimpl->useApproximatedHessian)) {
                m_pimpl->loader->Options()->SetStringValue("hessian_approximation", "limited-memory");
            }

            if (!(m_problem->info().hasNonLinearConstraints())) {
                m_pimpl->loader->Options()->SetStringValue("jac_c_constant", "yes");
                m_pimpl->loader->Options()->SetStringValue("jac_d_constant", "yes");

                if (m_problem->info().costIsQuadratic()) {
                    m_pimpl->loader->Options()->SetStringValue("hessian_constant", "yes");
                }
            }

            if (m_problem->info().hasSparseConstraintJacobian()) {
                if (!(m_problem->getConstraintsJacobianInfo(m_pimpl->nlpPointer->constraintsJacNNZRows,
                                                            m_pimpl->nlpPointer->constraintsJacNNZCols))){
                    reportError("IpoptInterface", "solve", "Error while retrieving constraints jacobian info.");
                    return false;
                }
            } else { //dense jacobian
                m_pimpl->nlpPointer->constraintsJacNNZCols.clear();
                m_pimpl->nlpPointer->constraintsJacNNZRows.clear();
                for (unsigned int i = 0; i < m_pimpl->nlpPointer->numberOfConstraints; i++) {
                    for (unsigned int j = 0; j < m_pimpl->nlpPointer->numberOfVariables; ++j) {
                        m_pimpl->nlpPointer->constraintsJacNNZRows.push_back(i);
                        m_pimpl->nlpPointer->constraintsJacNNZCols.push_back(j);
                    }
                }
            }

            if (m_problem->info().hasSparseHessian()) {
                if (!(m_problem->getHessianInfo(m_pimpl->nlpPointer->inputHessianNNZRows,
                                                m_pimpl->nlpPointer->inputHessianNNZCols))){
                    reportError("IpoptInterface", "solve", "Error while retrieving hessian info.");
                    return false;
                }

                std::vector<size_t>& iRows = m_pimpl->nlpPointer->inputHessianNNZRows;
                std::vector<size_t>& jCols = m_pimpl->nlpPointer->inputHessianNNZCols;

                std::vector<size_t>& iRowsLT = m_pimpl->nlpPointer->lowerTriangularHessianNNZRows;
                std::vector<size_t>& jColsLT = m_pimpl->nlpPointer->lowerTriangularHessianNNZCols;

                size_t nnz = 0;
                for (size_t i = 0; i < iRows.size(); ++i) {
                    if (jCols[i] <= iRows[i]) { //taking only the lower triangular part
                        if (nnz < iRowsLT.size()) {
                            iRowsLT[nnz] = iRows[i];
                            jColsLT[nnz] = jCols[i];
                        } else {
                            iRowsLT.push_back(iRows[i]);
                            jColsLT.push_back(jCols[i]);
                        }
                        ++nnz;
                    }
                }
                iRowsLT.resize(nnz);
                jColsLT.resize(nnz);

            } else { //dense hessian
                m_pimpl->nlpPointer->lowerTriangularHessianNNZRows.clear();
                m_pimpl->nlpPointer->lowerTriangularHessianNNZCols.clear();
                for (unsigned int i = 0; i < m_pimpl->nlpPointer->numberOfVariables; i++) {
                    for (unsigned int j = 0; j <= i; ++j) {
                        m_pimpl->nlpPointer->lowerTriangularHessianNNZRows.push_back(i);
                        m_pimpl->nlpPointer->lowerTriangularHessianNNZCols.push_back(j);
                    }
                }
            }

            if (m_pimpl->possibleReOptimize()){ //reoptimize possibility
                m_pimpl->loader->Options()->SetStringValueIfUnset("warm_start_init_point", "yes");
                m_pimpl->loader->Options()->SetStringValueIfUnset("warm_start_same_structure", "yes");
                m_pimpl->loader->Options()->SetNumericValueIfUnset("warm_start_bound_frac", 1e-6);
                m_pimpl->loader->Options()->SetNumericValueIfUnset("warm_start_bound_push", 1e-6);
                m_pimpl->loader->Options()->SetNumericValueIfUnset("warm_start_mult_bound_push", 1e-6);
                m_pimpl->loader->Options()->SetNumericValueIfUnset("warm_start_slack_bound_frac", 1e-6);
                m_pimpl->loader->Options()->SetNumericValueIfUnset("warm_start_slack_bound_push", 1e-6);

                m_pimpl->loader->ReOptimizeTNLP(m_pimpl->nlpPointer);
                if (m_pimpl->nlpPointer->exitCode < 0) {
                    return false;
                }
            } else {
                m_pimpl->loader->OptimizeTNLP(m_pimpl->nlpPointer);
                if (m_pimpl->nlpPointer->exitCode < 0) {
                    return false;
                }
            }

            m_pimpl->previousNumberOfVariables = m_pimpl->nlpPointer->numberOfVariables;
            m_pimpl->previousNumberOfConstraints = m_pimpl->nlpPointer->numberOfConstraints;
            m_pimpl->previousJacobianNonZeros = m_pimpl->nlpPointer->constraintsJacNNZRows.size();
            m_pimpl->previousHessianNonZeros = m_pimpl->nlpPointer->inputHessianNNZRows.size();

            return true;
        }

        bool IpoptInterface::getPrimalVariables(VectorDynSize &primalVariables)
        {
            if (m_pimpl->nlpPointer->exitCode == -6){
                reportError("IpoptInterface", "getPrimalVariables", "First you have to call the solve method.");
                return false;
            }

            if (m_pimpl->nlpPointer->exitCode < 0){
                reportError("IpoptInterface", "getPrimalVariables", "Cannot specify the primal variables if the problem has not been solved correctly.");
                return false;
            }

            primalVariables = m_pimpl->nlpPointer->solution;
            return true;
        }

        bool IpoptInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
        {
            if (m_pimpl->nlpPointer->exitCode == -6){
                reportError("IpoptInterface", "getDualVariables", "First you have to call the solve method.");
                return false;
            }

            if (m_pimpl->nlpPointer->exitCode < 0){
                reportError("IpoptInterface", "getDualVariables", "Cannot specify the dual variables if the problem has not been solved correctly.");
                return false;
            }

            constraintsMultipliers = m_pimpl->nlpPointer->constraintMultipliers;
            lowerBoundsMultipliers = m_pimpl->nlpPointer->lowerBoundMultipliers;
            upperBoundsMultipliers = m_pimpl->nlpPointer->upperBoundMultipliers;

            return true;
        }

        bool IpoptInterface::getOptimalCost(double &optimalCost)
        {
            if (m_pimpl->nlpPointer->exitCode == -6){
                reportError("IpoptInterface", "getOptimalCost", "First you have to call the solve method.");
                return false;
            }

            if (m_pimpl->nlpPointer->exitCode < 0){
                reportError("IpoptInterface", "getOptimalCost", "Cannot specify the optimal cost value if the problem has not been solved correctly.");
                return false;
            }

            optimalCost = m_pimpl->nlpPointer->optimalCostValue;
            return true;
        }

        bool IpoptInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
        {
            if (m_pimpl->nlpPointer->exitCode == -6){
                reportError("IpoptInterface", "getOptimalConstraintsValues", "First you have to call the solve method.");
                return false;
            }

            if (m_pimpl->nlpPointer->exitCode < 0){
                reportError("IpoptInterface", "getOptimalConstraintsValues", "Cannot specify the optimal constraints values if the problem has not been solved correctly.");
                return false;
            }

            constraintsValues = m_pimpl->nlpPointer->optimalConstraintsValue;
            return true;
        }

        double IpoptInterface::minusInfinity()
        {
            Ipopt::Number output = 1;
            m_pimpl->loader->Options()->GetNumericValue("nlp_lower_bound_inf", output, "");
            if (output > 0){
                reportWarning("IpoptInterface", "minusInfinity", "Error while reading the nlp_lower_bound_inf value from Ipopt. Returning the default.");
                return Optimizer::minusInfinity();
            }
            return static_cast<double>(output);
        }

        double IpoptInterface::plusInfinity()
        {
            Ipopt::Number output = -1;
            m_pimpl->loader->Options()->GetNumericValue("nlp_upper_bound_inf", output, "");
            if (output < 0){
                reportWarning("IpoptInterface", "minusInfinity", "Error while reading the nlp_upper_bound_inf value from Ipopt. Returning the default.");
                return Optimizer::plusInfinity();
            }
            return static_cast<double>(output);
        }

        void IpoptInterface::useApproximatedHessians(bool useApproximatedHessian)
        {
            m_pimpl->useApproximatedHessian = useApproximatedHessian;
        }

        bool IpoptInterface::setIpoptOption(const std::string &tag, const std::string &value)
        {
            return m_pimpl->loader->Options()->SetStringValue(tag, value);
        }

        bool IpoptInterface::setIpoptOption(const std::string &tag, double value)
        {
            return m_pimpl->loader->Options()->SetNumericValue(tag, value);
        }

        bool IpoptInterface::setIpoptOption(const std::string &tag, int value)
        {
            return m_pimpl->loader->Options()->SetIntegerValue(tag, value);
        }

        bool IpoptInterface::getIpoptOption(const std::string &tag, std::string &value)
        {
            return m_pimpl->loader->Options()->GetStringValue(tag, value, "");
        }

        bool IpoptInterface::getIpoptOption(const std::string &tag, double &value)
        {
            return m_pimpl->loader->Options()->GetNumericValue(tag, value, "");
        }

        bool IpoptInterface::getIpoptOption(const std::string &tag, int &value)
        {
            return m_pimpl->loader->Options()->GetIntegerValue(tag, value, "");
        }


    }
}
