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

#include <iDynTree/Optimizers/AlglibInterface.h>

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>

#include <optimization.h>
#include <stdafx.h>

#include <cassert>
#include <map>

namespace iDynTree {

    namespace optimization {

        typedef struct{
            double boundValue;
            double constraintSign;
            unsigned int index;
        }ConstraintInfo;

        typedef struct{
            std::shared_ptr<OptimizationProblem> problem;
            std::map<size_t, ConstraintInfo> expandedToOriginalInequalities, expandedToOriginalEqualities;
            VectorDynSize variablesBuffer, costJacobianBuffer, constraintsBuffer;
            MatrixDynSize constraintJacobian, zeroMatrix, jacBuffer;
        }SharedData;

        class AlglibInterface::AlglibInterfaceImplementation {
        public:
            SharedData* nlpData;
            VectorDynSize constraintsLowerBounds, constraintsUpperBounds, variablesLowerBounds, variablesUpperBounds, x0, solution;
            alglib::minnlcstate alglibState;
            alglib::minnlcreport alglibReport;
            alglib::real_1d_array alglibScaling;
            int exitCode;
            double epsx;
            alglib::ae_int_t maxits;
            alglib::ae_int_t outerits;
            double rho;


            AlglibInterfaceImplementation(){
                nlpData = new(SharedData);
                assert(nlpData);
                nlpData->problem = nullptr;
                exitCode = -10;
                epsx = 0.000001;
                maxits = 0;
                outerits = 5;
                rho = 1000;
            }

        };

        void ALGLIB_NLP(const alglib::real_1d_array &x, alglib::real_1d_array &fi, alglib::real_2d_array &jac, void *ptr){
            // See http://www.alglib.net/translator/man/manual.cpp.html#example_minnlc_d_inequality
            SharedData* pimpl = reinterpret_cast<SharedData*>(ptr);
            assert(pimpl->problem);
            Eigen::Map<const Eigen::VectorXd> x_map(x.getcontent(), x.length());
            Eigen::Map<Eigen::VectorXd> f_map(fi.getcontent(), fi.length());

            assert(x_map.size() == pimpl->problem->numberOfVariables());
            assert(f_map.size() == pimpl->expandedToOriginalInequalities.size() + pimpl->expandedToOriginalEqualities.size() + 1); //cost function + constraints
            assert((jac.rows() == f_map.size()) && (jac.cols() == x_map.size()));
            assert((jac.rows() == pimpl->jacBuffer.rows()) && (jac.cols() == pimpl->jacBuffer.cols()));

            toEigen(pimpl->variablesBuffer) = x_map;

            if (!(pimpl->problem->setVariables(pimpl->variablesBuffer))){
                reportError("ALGLIBInterface", "solve", "Error while setting the optimizatin variables to the problem.");
                assert(false);
            }

            double cost = 0;
            if (!(pimpl->problem->evaluateCostFunction(cost))){
                reportError("ALGLIBInterface", "solve", "Error while evaluating the cost function.");
                assert(false);
            }

            if (!(pimpl->problem->evaluateCostGradient(pimpl->costJacobianBuffer))){
                reportError("ALGLIBInterface", "solve", "Error while retrieving the cost gradient.");
                assert(false);
            }

            if (!(pimpl->problem->evaluateConstraints(pimpl->constraintsBuffer))){
                reportError("ALGLIBInterface", "solve", "Error while evaluating the constraints.");
                assert(false);
            }

            pimpl->constraintJacobian = pimpl->zeroMatrix; //initialization for nonzero elements

            if (!(pimpl->problem->evaluateConstraintsJacobian(pimpl->constraintJacobian))){
                reportError("ALGLIBInterface", "solve", "Error while evaluating the constraints jacobian.");
                assert(false);
            }

            f_map(0) = cost;
            toEigen(pimpl->jacBuffer).row(0) = toEigen(pimpl->costJacobianBuffer);
            long index = 1;
            ConstraintInfo constraint;
            for (size_t i = 0; i < pimpl->expandedToOriginalEqualities.size(); ++i){
                constraint = pimpl->expandedToOriginalEqualities[i];
                f_map(index) = constraint.boundValue + constraint.constraintSign * pimpl->constraintsBuffer(constraint.index);
                toEigen(pimpl->jacBuffer).row(index) = constraint.constraintSign * toEigen(pimpl->constraintJacobian).row(constraint.index);
                index++;
            }

            for (size_t i = 0; i < pimpl->expandedToOriginalInequalities.size(); ++i){
                constraint = pimpl->expandedToOriginalInequalities[i];
                f_map(index) = constraint.boundValue + constraint.constraintSign * pimpl->constraintsBuffer(constraint.index);
                toEigen(pimpl->jacBuffer).row(index) = constraint.constraintSign * toEigen(pimpl->constraintJacobian).row(constraint.index);
                index++;
            }

            alglib::real_2d_array tempAlglibMatrix;
            tempAlglibMatrix.attach_to_ptr(pimpl->jacBuffer.rows(), pimpl->jacBuffer.cols(), pimpl->jacBuffer.data());
            jac = tempAlglibMatrix;

        }




        AlglibInterface::AlglibInterface()
        :m_pimpl(new AlglibInterfaceImplementation())
        {
            assert(m_pimpl);
        }

        AlglibInterface::~AlglibInterface()
        {
            if (m_pimpl){
                if (m_pimpl->nlpData){
                    delete m_pimpl->nlpData;
                    m_pimpl->nlpData = nullptr;
                }
                delete m_pimpl;
                m_pimpl = nullptr;
            }

        }

        bool AlglibInterface::isAvailable() const
        {
            return true;
        }

        bool AlglibInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
        {
            if (!problem){
                reportError("ALGLIBInterface", "setProblem", "Empty problem pointer.");
                return false;
            }
            m_pimpl->nlpData->problem = problem;
            return true;
        }

        bool AlglibInterface::solve()
        {
            if (!(m_pimpl->nlpData->problem)){
                reportError("ALGLIBInterface", "solve", "First you have to set the problem.");
                return false;
            }

            if (!(m_pimpl->nlpData->problem->prepare())){
                reportError("ALGLIBInterface", "solve", "Failed while preparing the optimization problem.");
                return false;
            }

            unsigned int nx = m_pimpl->nlpData->problem->numberOfVariables();
            unsigned int nc = m_pimpl->nlpData->problem->numberOfConstraints();

            if (m_pimpl->nlpData->variablesBuffer.size() != nx)
                m_pimpl->nlpData->variablesBuffer.resize(nx);

            if (m_pimpl->solution.size() != nx)
                m_pimpl->solution.resize(nx);

            if (m_pimpl->nlpData->constraintsBuffer.size() != nc)
                m_pimpl->nlpData->constraintsBuffer.resize(nc);

            if (m_pimpl->nlpData->costJacobianBuffer.size() != nx)
                m_pimpl->nlpData->costJacobianBuffer.resize(nx);

            if ((m_pimpl->nlpData->constraintJacobian.rows() != nc) || (m_pimpl->nlpData->constraintJacobian.cols() != nx))
                m_pimpl->nlpData->constraintJacobian.resize(nc, nx);

            if ((m_pimpl->nlpData->zeroMatrix.rows() != nc) || (m_pimpl->nlpData->zeroMatrix.cols() != nx)){
                m_pimpl->nlpData->zeroMatrix.resize(nc, nx);
                m_pimpl->nlpData->zeroMatrix.zero();
            }

            if (m_pimpl->variablesLowerBounds.size() != nx)
                m_pimpl->variablesLowerBounds.resize(nx);

            if (!(m_pimpl->nlpData->problem->getVariablesLowerBound(m_pimpl->variablesLowerBounds)))
                toEigen(m_pimpl->variablesLowerBounds).setConstant(minusInfinity());

            if (m_pimpl->variablesUpperBounds.size() != nx)
                m_pimpl->variablesUpperBounds.resize(nx);

            if (!(m_pimpl->nlpData->problem->getVariablesUpperBound(m_pimpl->variablesUpperBounds)))
                toEigen(m_pimpl->variablesUpperBounds).setConstant(plusInfinity());

            if (m_pimpl->constraintsUpperBounds.size() != nc)
                m_pimpl->constraintsUpperBounds.resize(nc);

            if (m_pimpl->constraintsLowerBounds.size() != nc)
                m_pimpl->constraintsLowerBounds.resize(nc);

            if (!(m_pimpl->nlpData->problem->getConstraintsBounds(m_pimpl->constraintsLowerBounds, m_pimpl->constraintsUpperBounds))){
                reportError("ALGLIBInterface", "solve", "Error while retrieving the constraints bounds.");
                return false;
            }

            unsigned int inequalities = 0, equalities = 0;
            ConstraintInfo newConstraint;
            for (unsigned int i=0; i < nc; ++i){
                if (checkDoublesAreEqual(m_pimpl->constraintsLowerBounds(i), m_pimpl->constraintsUpperBounds(i))){
                    newConstraint.boundValue = -1.0 * m_pimpl->constraintsUpperBounds(i);
                    newConstraint.constraintSign = +1.0;
                    newConstraint.index = i;
                    m_pimpl->nlpData->expandedToOriginalEqualities[equalities] = newConstraint;
                    equalities++;
                } else {

                    if (m_pimpl->constraintsLowerBounds(i) > minusInfinity()){
                        newConstraint.boundValue = m_pimpl->constraintsLowerBounds(i);
                        newConstraint.constraintSign = -1.0;
                        newConstraint.index = i;
                        m_pimpl->nlpData->expandedToOriginalInequalities[inequalities] = newConstraint;
                        inequalities++;
                    }

                    if (m_pimpl->constraintsUpperBounds(i) < plusInfinity()){
                        newConstraint.boundValue = -1.0 * m_pimpl->constraintsUpperBounds(i);
                        newConstraint.constraintSign = +1.0;
                        newConstraint.index = i;
                        m_pimpl->nlpData->expandedToOriginalInequalities[inequalities] = newConstraint;
                        inequalities++;
                    }
                }
            }
            m_pimpl->nlpData->expandedToOriginalEqualities.erase(m_pimpl->nlpData->expandedToOriginalEqualities.find(equalities), m_pimpl->nlpData->expandedToOriginalEqualities.end());
            m_pimpl->nlpData->expandedToOriginalInequalities.erase(m_pimpl->nlpData->expandedToOriginalInequalities.find(inequalities), m_pimpl->nlpData->expandedToOriginalInequalities.end());

            if ((m_pimpl->nlpData->jacBuffer.rows() != (equalities + inequalities + 1)) || (m_pimpl->nlpData->jacBuffer.cols() != nx))
                m_pimpl->nlpData->jacBuffer.resize((equalities + inequalities + 1), nx);

            // See http://www.alglib.net/translator/man/manual.cpp.html#sub_minnlccreate

            if (m_pimpl->x0.size() != nx){
                m_pimpl->x0.resize(nx);
                m_pimpl->x0.zero();
            }

            if (m_pimpl->nlpData->problem->getGuess(m_pimpl->x0)) {
                if (m_pimpl->x0.size() != nx) {
                    reportError("ALGLIBInterface", "solve", "The initial guess dimension does not match the problem dimension.");
                    return false;
                }
            }


            alglib::real_1d_array alglibX0, alglibStart, alglibSolution, alglibScaling, alglibLowerBound, alglibUpperBound;
            alglibX0.attach_to_ptr(m_pimpl->x0.size(), m_pimpl->x0.data());
            if (m_pimpl->alglibScaling.length() != nx)
                m_pimpl->alglibScaling.setlength(nx);
            Eigen::Map<Eigen::VectorXd> scaling_Map(m_pimpl->alglibScaling.getcontent(), m_pimpl->alglibScaling.length());
            scaling_Map.setConstant(1.0); //no scaling


            // Create optimizer object, choose AUL algorithm and tune its settings:
            // * rho=1000       penalty coefficient
            // * outerits=5     number of outer iterations to tune Lagrange coefficients
            // * epsx=0.000001  stopping condition for inner iterations
            // * s=[1,1]        all variables have unit scale
            // * exact low-rank preconditioner is used, updated after each 10 iterations
            //
            if ((m_pimpl->exitCode >= 0) && (m_pimpl->solution.size() == nx)){
                alglibStart.attach_to_ptr(m_pimpl->solution.size(), m_pimpl->solution.data());
                alglib::minnlcrestartfrom(m_pimpl->alglibState, alglibStart);
            } else {
                alglib::minnlccreate(nx, alglibX0, m_pimpl->alglibState);
            }

            alglib::minnlcsetalgoaul(m_pimpl->alglibState, m_pimpl->rho, m_pimpl->outerits);
            alglib::minnlcsetcond(m_pimpl->alglibState, m_pimpl->epsx, m_pimpl->maxits);
            alglib::minnlcsetscale(m_pimpl->alglibState, m_pimpl->alglibScaling);
            alglib::minnlcsetprecinexact(m_pimpl->alglibState);

            alglibLowerBound.attach_to_ptr(m_pimpl->variablesLowerBounds.size(), m_pimpl->variablesLowerBounds.data());
            alglibUpperBound.attach_to_ptr(m_pimpl->variablesUpperBounds.size(), m_pimpl->variablesUpperBounds.data());

            alglib::minnlcsetbc(m_pimpl->alglibState, alglibLowerBound, alglibUpperBound);
            alglib::minnlcsetnlc(m_pimpl->alglibState, m_pimpl->nlpData->expandedToOriginalEqualities.size(), m_pimpl->nlpData->expandedToOriginalInequalities.size());

            alglib::minnlcoptimize(m_pimpl->alglibState, ALGLIB_NLP, nullptr, m_pimpl->nlpData);

            alglibSolution.attach_to_ptr(m_pimpl->solution.size(), m_pimpl->solution.data());
            alglib::minnlcresultsbuf(m_pimpl->alglibState, alglibSolution, m_pimpl->alglibReport);
            m_pimpl->exitCode = m_pimpl->alglibReport.terminationtype;

            if (m_pimpl->exitCode < 0) {
                std::ostringstream errorMsg;
                errorMsg << "Optimization problem failed because ";
                switch(m_pimpl->exitCode){
                    case -3:
                        errorMsg << "constraints are inconsistent.";
                        break;

                    case -5:
                        errorMsg << "inappropriate solver was used.";
                        break;

                    case -7:
                        errorMsg << "derivative correctness check failed.";
                        break;

                    case -8:
                        errorMsg << "optimizer detected NAN/INF values either in the function itself, or in its Jacobian.";
                        break;

                    default:
                        errorMsg << "of an unknown error.";
                }
                reportError("ALGLIBInterface", "solve", errorMsg.str().c_str());
                return false;
            }

            return true;
        }

        bool AlglibInterface::getPrimalVariables(VectorDynSize &primalVariables)
        {
            if (m_pimpl->exitCode < 0){
                reportError("ALGLIBInterface", "getPrimalVariables", "Previous run was not successfull. Cannot retrieve primal variables.");
                return false;
            }
            primalVariables = m_pimpl->solution;
            return true;
        }

        bool AlglibInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
        {
            reportError("ALGLIBInterface", "getDualVariables", "ALGLIB does not give information about the dual variables.");
            return false;
        }

        bool AlglibInterface::getOptimalCost(double &optimalCost)
        {
            if (m_pimpl->exitCode < 0){
                reportError("ALGLIBInterface", "getOptimalCost", "Previous run was not successfull. Cannot retrieve optimal cost.");
                return false;
            }

            if (!(m_pimpl->nlpData->problem->setVariables(m_pimpl->solution))){
                reportError("ALGLIBInterface", "getOptimalCost", "Error while setting the optimal variables to the problem.");
                return false;
            }

            double cost = 0;
            if (!(m_pimpl->nlpData->problem->evaluateCostFunction(cost))){
                reportError("ALGLIBInterface", "getOptimalCost", "Error while evaluating the cost function.");
                return false;
            }
            optimalCost = cost;
            return true;
        }

        bool AlglibInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
        {
            if (m_pimpl->exitCode < 0){
                reportError("ALGLIBInterface", "getOptimalConstraintsValues", "Previous run was not successfull. Cannot retrieve optimal constraints value.");
                return false;
            }

            if (!(m_pimpl->nlpData->problem->setVariables(m_pimpl->solution))){
                reportError("ALGLIBInterface", "getOptimalConstraintsValues", "Error while setting the optimal variables to the problem.");
                return false;
            }

            if (!(m_pimpl->nlpData->problem->evaluateConstraints(m_pimpl->nlpData->variablesBuffer))){
                reportError("ALGLIBInterface", "getOptimalConstraintsValues", "Error while evaluating the constraints function.");
                return false;
            }
            constraintsValues = m_pimpl->nlpData->variablesBuffer;
            return true;
        }

        double AlglibInterface::minusInfinity()
        {
            return alglib::fp_neginf;
        }

        double AlglibInterface::plusInfinity()
        {
            return alglib::fp_posinf;
        }

        bool AlglibInterface::setRHO(double rho)
        {
            if (rho <= 0){
                reportError("ALGLIBInterface", "setRHO", "The rho parameter is supossed to be positive;");
            }
            m_pimpl->rho = rho;
            return true;
        }

        bool AlglibInterface::setOuterIterations(unsigned int outerIterations)
        {
            m_pimpl->outerits = outerIterations;
            return true;
        }

        bool AlglibInterface::setStoppingCondition(double epsX)
        {
            if (epsX < 0){
                reportError("ALGLIBInterface", "setStoppingCondition", "The epsX parameter is supposed to be non negative");
                return false;
            }

            m_pimpl->epsx = epsX;

            return true;
        }

        bool AlglibInterface::setMaximumIterations(unsigned int maxIter)
        {
            m_pimpl->maxits = maxIter;
            return true;
        }

    }

}

